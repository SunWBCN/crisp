// TCP/compliance-frame implementation of runCartesianAdmittanceToTarget
// (declared in my_controller/cartesian_admittance.hpp). The nominal TCP
// trajectory stays in Franka's O/base frame. External wrench, virtual M/D/K,
// compliant displacement and velocity are evaluated in one explicitly selected
// compliance frame (actual_tcp, nominal_tcp or world). The compliant
// displacement is rotated back into O before it is composed with the nominal
// trajectory. No main() lives here so this translation unit can replace the
// original one in the existing executable without changing its public API.
// pixi run -e jazzy-crisp ros2 run my_controller
// cartesian_admittance 10.90.90.10 \ --wrench-source serial --serial-port
// /dev/ttyACM0 --wrench-frame local --wrench-sign 1.0 \ --target-pose-source
// home --target-ramp-time 3.0 \ --adm-mask 0,0,1,0,0,0 --adm-mass 3,3,3,1,1,1
// --adm-stiffness 0,0,80,0,0,0 --adm-damping 0,0,31,0,0,0 \ --stop-on-settled
// false \ --log-pose-error false --print-pose-error true --print-wrench-debug
// false
#include "my_controller/cartesian_admittance.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;

enum class ComplianceFrame {
  kActualTcp,
  kNominalTcp,
  kWorld,
};

// Joint-space impedance gains are now configured through the YAML file and
// carried in CartesianAdmittanceConfig::joint_stiffness / joint_damping (their
// defaults preserve the previously hard-coded values). The joint-impedance
// torque callback reads those fixed-size std::array<double, 7> fields directly
// -- no allocation, no file I/O in the RT loop.

constexpr int kWrenchBiasSamples = 500;
constexpr size_t kWrenchHistorySize =
    2000; // ~2 s at 1 kHz, retained for reflex post-mortem
constexpr double kRadToDeg = 57.295779513082320876;
constexpr size_t kPoseLogMaxCapacity =
    120000; // ~120 s at 1 kHz, preallocated ring buffer
constexpr size_t kFilterLogMaxCapacity =
    120000; // ~120 s at 1 kHz, preallocated ring buffer
constexpr double kTargetStationaryLinearSpeed = 1.0e-5;  // m/s
constexpr double kTargetStationaryAngularSpeed = 1.0e-4; // rad/s

// Fixed output location for the unfiltered-vs-filtered force comparison, plus
// the Python script that renders the four-subplot PNG. Both are absolute paths
// in the source tree so the feature works regardless of the shell's working
// directory (matching the wrapper's kDefaultConfigPath convention).
constexpr const char *kFilterComparisonDir =
    "/home/wenbin/github_repo/crisp_franka/src/my_controller/scripts/"
    "wrench_filter_comparison";
constexpr const char *kFilterComparisonPlotScript =
    "/home/wenbin/github_repo/crisp_franka/src/my_controller/scripts/"
    "plot_wrench_filter_comparison.py";

struct WrenchDebugData {
  std::array<std::atomic<double>, 6> raw{};
  std::array<std::atomic<double>, 6> bias_removed{};
  std::array<std::atomic<double>, 6> masked{};
  std::array<std::atomic<double>, 6> deadbanded{};
  std::array<std::atomic<double>, 6> filtered{};
  std::atomic<int> bias_count{0};
  std::atomic<bool> has_sample{false};
};

void storeVector(std::array<std::atomic<double>, 6> &target,
                 const Vector6d &source) {
  for (int i = 0; i < 6; ++i) {
    target[static_cast<size_t>(i)].store(source(i), std::memory_order_relaxed);
  }
}

Vector6d loadVector(const std::array<std::atomic<double>, 6> &source) {
  Vector6d value;
  for (int i = 0; i < 6; ++i) {
    value(i) = source[static_cast<size_t>(i)].load(std::memory_order_relaxed);
  }
  return value;
}

// One control-cycle snapshot of the wrench pipeline, kept in a ring buffer so
// the samples leading up to a reflex/error can be dumped after control stops.
struct WrenchSample {
  double t = 0.0;
  int bias_count = 0;
  std::array<double, 6> raw{};
  std::array<double, 6> bias_removed{};
  std::array<double, 6> masked{};
  std::array<double, 6> deadbanded{};
  std::array<double, 6> filtered{};
};

std::array<double, 6> toArray(const Vector6d &v) {
  std::array<double, 6> result{};
  for (int i = 0; i < 6; ++i) {
    result[static_cast<size_t>(i)] = v(i);
  }
  return result;
}

// Called from the catch handler (control has already stopped, so no concurrent
// writer): prints the last sample and writes the retained history to a
// timestamped CSV.
void dumpWrenchHistory(const std::vector<WrenchSample> &history, size_t count,
                       const std::string &reason) {
  if (count == 0) {
    std::cerr << "No wrench samples were recorded before the stop."
              << std::endl;
    return;
  }
  const size_t retained = std::min(count, history.size());
  const size_t start = count - retained;
  const WrenchSample &last = history[(count - 1) % history.size()];

  std::cerr << std::fixed << std::setprecision(3)
            << "Last wrench before stop @t=" << last.t << "s | raw F[N]=["
            << last.raw[0] << ", " << last.raw[1] << ", " << last.raw[2]
            << "] T[Nm]=[" << last.raw[3] << ", " << last.raw[4] << ", "
            << last.raw[5] << "] | filtered F[N]=[" << last.filtered[0] << ", "
            << last.filtered[1] << ", " << last.filtered[2] << "] T[Nm]=["
            << last.filtered[3] << ", " << last.filtered[4] << ", "
            << last.filtered[5] << "]" << std::endl;

  std::time_t now = std::time(nullptr);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
  const std::string path =
      std::string("wrench_before_reflex_") + stamp + ".csv";

  std::ofstream out(path);
  if (!out) {
    std::cerr << "Could not open " << path << " to save the wrench history."
              << std::endl;
    return;
  }
  out << std::fixed << std::setprecision(6);
  out << "# reason: " << reason << "\n";
  out << "t,bias_count,"
         "raw_fx,raw_fy,raw_fz,raw_tx,raw_ty,raw_tz,"
         "bias_removed_fx,bias_removed_fy,bias_removed_fz,bias_removed_tx,bias_"
         "removed_ty,bias_removed_tz,"
         "masked_fx,masked_fy,masked_fz,masked_tx,masked_ty,masked_tz,"
         "deadbanded_fx,deadbanded_fy,deadbanded_fz,deadbanded_tx,deadbanded_"
         "ty,deadbanded_tz,"
         "filtered_fx,filtered_fy,filtered_fz,filtered_tx,filtered_ty,filtered_"
         "tz\n";
  for (size_t k = start; k < count; ++k) {
    const WrenchSample &s = history[k % history.size()];
    out << s.t << "," << s.bias_count;
    for (double v : s.raw)
      out << "," << v;
    for (double v : s.bias_removed)
      out << "," << v;
    for (double v : s.masked)
      out << "," << v;
    for (double v : s.deadbanded)
      out << "," << v;
    for (double v : s.filtered)
      out << "," << v;
    out << "\n";
  }
  std::cerr << "Recorded " << retained
            << " wrench samples before the stop to ./" << path << std::endl;
}

void printWrenchLine(const char *label, const Vector6d &wrench) {
  std::cout << label << " F[N]=[" << wrench(0) << ", " << wrench(1) << ", "
            << wrench(2) << "] T[Nm]=[" << wrench(3) << ", " << wrench(4)
            << ", " << wrench(5) << "]";
}

void printExternalForceLine(const Vector6d &wrench) {
  const double force_norm = wrench.head<3>().norm();
  std::cout << "F_ext[N]=[" << wrench(0) << ", " << wrench(1) << ", "
            << wrench(2) << "] |F|=" << force_norm << " N";
}

Eigen::Affine3d arrayToTransform(const std::array<double, 16> &array) {
  Eigen::Affine3d transform(Eigen::Affine3d::Identity());
  transform.matrix() =
      Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(
          array.data());
  return transform;
}

std::array<double, 16> transformToArray(const Eigen::Vector3d &position,
                                        const Eigen::Matrix3d &rotation) {
  Eigen::Matrix<double, 4, 4, Eigen::ColMajor> transform =
      Eigen::Matrix<double, 4, 4, Eigen::ColMajor>::Identity();
  transform.template topLeftCorner<3, 3>() = rotation;
  transform.template topRightCorner<3, 1>() = position;

  std::array<double, 16> array{};
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(array.data()) =
      transform;
  return array;
}

Eigen::Vector3d rotationLog(const Eigen::Matrix3d &rotation) {
  Eigen::Quaterniond q(rotation);
  q.normalize();
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }

  const Eigen::Vector3d vec(q.x(), q.y(), q.z());
  const double vec_norm = vec.norm();
  if (vec_norm < 1.0e-9) {
    return Eigen::Vector3d::Zero();
  }

  const double angle = 2.0 * std::atan2(vec_norm, q.w());
  return angle * vec / vec_norm;
}

Eigen::Matrix3d integrateRotation(const Eigen::Matrix3d &rotation,
                                  const Eigen::Vector3d &delta) {
  const double angle = delta.norm();
  if (angle < 1.0e-12) {
    return rotation;
  }

  Eigen::Matrix3d updated =
      Eigen::AngleAxisd(angle, delta / angle).toRotationMatrix() * rotation;
  Eigen::Quaterniond normalized(updated);
  normalized.normalize();
  return normalized.toRotationMatrix();
}

void limitVectorNorm(Eigen::Vector3d &value, double max_norm) {
  const double norm = value.norm();
  if (norm > max_norm && norm > 1.0e-12) {
    value *= max_norm / norm;
  }
}

// Continuous radial deadband. Unlike simply zeroing the vector below the
// threshold and passing the full vector above it, this has no jump at |value|
// == threshold.
void applyVectorNormDeadband(Eigen::Vector3d &value, double threshold) {
  const double norm = value.norm();
  if (norm <= threshold || norm < 1.0e-12) {
    value.setZero();
    return;
  }
  value *= (norm - threshold) / norm;
}

Eigen::Matrix<double, 6, 6>
diagonalMatrix(const std::array<double, 6> &values) {
  Eigen::Matrix<double, 6, 6> matrix = Eigen::Matrix<double, 6, 6>::Zero();
  for (int i = 0; i < 6; ++i) {
    matrix(i, i) = values[static_cast<size_t>(i)];
  }
  return matrix;
}

ComplianceFrame parseComplianceFrame(const std::string &frame) {
  if (frame == "actual_tcp") {
    return ComplianceFrame::kActualTcp;
  }
  if (frame == "nominal_tcp" || frame == "desired_tcp") {
    return ComplianceFrame::kNominalTcp;
  }
  if (frame == "world") {
    return ComplianceFrame::kWorld;
  }
  throw std::invalid_argument(
      "admittance.frame must be one of: actual_tcp, nominal_tcp, world");
}

const char *complianceFrameName(ComplianceFrame frame) {
  switch (frame) {
  case ComplianceFrame::kActualTcp:
    return "actual_tcp";
  case ComplianceFrame::kNominalTcp:
    return "nominal_tcp";
  case ComplianceFrame::kWorld:
    return "world";
  }
  return "unknown";
}

// Minimum-jerk scaling s(t) in [0, 1] used to ramp the desired pose from start
// to target: s = 10*tau^3 - 15*tau^4 + 6*tau^5 with tau = clamp(t/duration, 0,
// 1). Zero velocity and acceleration at both ends, so the transition never
// produces a sudden command.
double minimumJerkScale(double t, double duration) {
  if (duration <= 0.0) {
    return 1.0;
  }
  const double tau = std::clamp(t / duration, 0.0, 1.0);
  return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
}

class RealtimeInputs : public rclcpp::Node {
public:
  explicit RealtimeInputs(const std::string &wrench_topic)
      : Node("libfranka_cartesian_admittance") {
    for (auto &value : wrench_) {
      value.store(0.0, std::memory_order_relaxed);
    }

    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
        wrench_topic, rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
          wrench_[0].store(msg->wrench.force.x, std::memory_order_relaxed);
          wrench_[1].store(msg->wrench.force.y, std::memory_order_relaxed);
          wrench_[2].store(msg->wrench.force.z, std::memory_order_relaxed);
          wrench_[3].store(msg->wrench.torque.x, std::memory_order_relaxed);
          wrench_[4].store(msg->wrench.torque.y, std::memory_order_relaxed);
          wrench_[5].store(msg->wrench.torque.z, std::memory_order_relaxed);
          has_wrench_.store(true, std::memory_order_release);
        });

    RCLCPP_INFO(get_logger(), "Reading wrench from '%s'.",
                wrench_topic.c_str());
  }

  Vector6d wrench() const {
    Vector6d value;
    for (int i = 0; i < 6; ++i) {
      value(i) =
          wrench_[static_cast<size_t>(i)].load(std::memory_order_relaxed);
    }
    return value;
  }

  bool hasWrench() const { return has_wrench_.load(std::memory_order_acquire); }

private:
  std::array<std::atomic<double>, 6> wrench_{};
  std::atomic<bool> has_wrench_{false};
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      wrench_sub_;
};

class SerialWrenchReader {
public:
  explicit SerialWrenchReader(const std::string &port) : port_(port) {
    for (auto &value : wrench_) {
      value.store(0.0, std::memory_order_relaxed);
    }
    openPort();
    read_thread_ = std::thread([this]() { readLoop(); });
  }

  ~SerialWrenchReader() {
    running_.store(false, std::memory_order_release);
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  Vector6d wrench() const {
    Vector6d value;
    for (int i = 0; i < 6; ++i) {
      value(i) =
          wrench_[static_cast<size_t>(i)].load(std::memory_order_relaxed);
    }
    return value;
  }

  bool hasWrench() const { return has_wrench_.load(std::memory_order_acquire); }

private:
  void openPort() {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      throw std::runtime_error("Failed to open serial port '" + port_ +
                               "': " + std::strerror(errno));
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      throw std::runtime_error("Failed to read serial settings for '" + port_ +
                               "': " + std::strerror(errno));
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, B2000000);
    cfsetospeed(&tty, B2000000);
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    tty.c_cc[VMIN] = 28;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      throw std::runtime_error("Failed to configure serial port '" + port_ +
                               "': " + std::strerror(errno));
    }
    tcflush(fd_, TCIFLUSH);
  }

  bool readExact(std::array<unsigned char, 28> &buffer) {
    size_t offset = 0;
    while (running_.load(std::memory_order_acquire) && offset < buffer.size()) {
      const ssize_t n =
          ::read(fd_, buffer.data() + offset, buffer.size() - offset);
      if (n > 0) {
        offset += static_cast<size_t>(n);
      } else if (n < 0) {
        return false;
      }
    }
    return offset == buffer.size();
  }

  void readLoop() {
    std::array<unsigned char, 28> frame{};
    while (running_.load(std::memory_order_acquire)) {
      if (!readExact(frame)) {
        continue;
      }

      for (int i = 0; i < 6; ++i) {
        float value = 0.0F;
        std::memcpy(&value,
                    frame.data() + static_cast<size_t>(i) * sizeof(float),
                    sizeof(float));
        wrench_[static_cast<size_t>(i)].store(static_cast<double>(value),
                                              std::memory_order_relaxed);
      }
      has_wrench_.store(true, std::memory_order_release);
    }
  }

  std::string port_;
  int fd_{-1};
  std::array<std::atomic<double>, 6> wrench_{};
  std::atomic<bool> has_wrench_{false};
  std::atomic<bool> running_{true};
  std::thread read_thread_;
};

// Return the Franka-estimated wrench exactly in its configured source
// representation. K_F_ext_hat_K is expressed in K axes; O_F_ext_hat_K is
// expressed in O axes. Both are referenced at stiffness-frame origin K and are
// shifted to the physical TCP later.
Vector6d frankaWrenchSource(const franka::RobotState &state,
                            const std::string &wrench_frame) {
  Vector6d wrench;
  const auto &source =
      wrench_frame == "local" ? state.K_F_ext_hat_K : state.O_F_ext_hat_K;
  for (int i = 0; i < 6; ++i) {
    wrench(i) = source[static_cast<size_t>(i)];
  }
  return wrench;
}

// ---- Pose-error monitoring, settling detection and CSV logging (terminal
// tuning) ----

std::array<double, 3> toArray3(const Eigen::Vector3d &v) {
  return {v.x(), v.y(), v.z()};
}

void storeVector3(std::array<std::atomic<double>, 3> &target,
                  const Eigen::Vector3d &source) {
  for (int i = 0; i < 3; ++i) {
    target[static_cast<size_t>(i)].store(source(i), std::memory_order_relaxed);
  }
}

// Latest pose error, shared with the periodic printer thread; written from the
// RT callback.
struct PoseDebugData {
  std::array<std::atomic<double>, 3> actual{};
  std::array<std::atomic<double>, 3> position_error{};
  std::atomic<double> position_error_norm{0.0};
  std::array<std::atomic<double>, 3> orientation_error{};
  std::atomic<double> orientation_error_norm_rad{0.0};
  std::atomic<double> orientation_error_norm_deg{0.0};
  // Human-readable RPY breakdown in degrees (convention R =
  // Rz(yaw)*Ry(pitch)*Rx(roll)).
  std::array<std::atomic<double>, 3> orientation_error_rpy_deg{};
  std::array<std::atomic<double>, 3> actual_rpy_deg{};
  std::array<std::atomic<double>, 3> target_rpy_deg{};
  std::atomic<bool> has_sample{false};
};

// One row of the pose-error log, kept in a preallocated ring buffer (no RT
// allocation).
struct PoseSample {
  double t = 0.0;
  std::array<double, 3> target{};
  std::array<double, 3> actual{};
  // Compliant TCP command in world coordinates (legacy CSV column name:
  // inner_*).
  std::array<double, 3> inner{};
  std::array<double, 3> position_error{};
  double position_error_norm = 0.0;
  std::array<double, 3> orientation_error{};
  double orientation_error_norm_rad = 0.0;
  double orientation_error_norm_deg = 0.0;
  std::array<double, 3> target_rpy_deg{}; // [roll, pitch, yaw]
  std::array<double, 3> actual_rpy_deg{}; // [roll, pitch, yaw]
  std::array<double, 3>
      error_rpy_deg{}; // wrapped target - actual, [roll, pitch, yaw]
  std::array<double, 3> inner_velocity{}; // selected compliance-frame axes
  std::array<double, 3> filtered_force{}; // selected compliance-frame axes
  std::array<double, 3> masked_force{};   // selected compliance-frame axes
};

struct PoseError {
  Eigen::Vector3d position_error = Eigen::Vector3d::Zero();
  double position_error_norm = 0.0;
  Eigen::Vector3d orientation_error =
      Eigen::Vector3d::Zero(); // rotation-log vector (rad)
  double orientation_error_norm_rad = 0.0;
  double orientation_error_norm_deg = 0.0;
  // Human-readable RPY breakdown (rad), convention R =
  // Rz(yaw)*Ry(pitch)*Rx(roll). Debug only -- the rotation-log fields above
  // stay the true orientation error / its norm.
  Eigen::Vector3d target_rpy = Eigen::Vector3d::Zero();
  Eigen::Vector3d actual_rpy = Eigen::Vector3d::Zero();
  Eigen::Vector3d orientation_error_rpy =
      Eigen::Vector3d::Zero(); // wrap(target_rpy - actual_rpy)
};

// Wrap an angle to (-pi, pi] with no branching/allocation (RT-safe).
double wrapToPi(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

// e_p = p_actual - p_target, e_R = log(R_target * R_actual^T) (the true
// orientation error). Also fills a per-axis RPY breakdown -- target, actual,
// and their wrapped difference -- for human-readable debugging; this is NOT the
// rotation-log vector. RT-safe (stack only).
PoseError computePoseError(const Eigen::Vector3d &target_position,
                           const Eigen::Matrix3d &target_rotation,
                           const Eigen::Vector3d &actual_position,
                           const Eigen::Matrix3d &actual_rotation) {
  PoseError error;
  error.position_error = actual_position - target_position;
  error.position_error_norm = error.position_error.norm();
  error.orientation_error =
      rotationLog(target_rotation * actual_rotation.transpose());
  error.orientation_error_norm_rad = error.orientation_error.norm();
  error.orientation_error_norm_deg =
      error.orientation_error_norm_rad * kRadToDeg;
  error.target_rpy = my_controller::rotationMatrixToRPY(target_rotation);
  error.actual_rpy = my_controller::rotationMatrixToRPY(actual_rotation);
  for (int i = 0; i < 3; ++i) {
    error.orientation_error_rpy(i) =
        wrapToPi(error.target_rpy(i) - error.actual_rpy(i));
  }
  return error;
}

void printSettledSummary(const Eigen::Vector3d &target_position,
                         const Eigen::Vector3d &stable_position,
                         const PoseError &error) {
  std::cout << "\n===== Settled Pose Error Summary =====\n";
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "Target TCP position [m]:        [" << target_position.x()
            << ", " << target_position.y() << ", " << target_position.z()
            << "]\n";
  std::cout << "Stable TCP position [m]:        [" << stable_position.x()
            << ", " << stable_position.y() << ", " << stable_position.z()
            << "]\n";
  std::cout << "Position error [m]:             [" << error.position_error.x()
            << ", " << error.position_error.y() << ", "
            << error.position_error.z() << "]\n";
  std::cout << "Position error norm [m]:        " << error.position_error_norm
            << "\n\n";
  std::cout << std::setprecision(3);
  std::cout << "Orientation error vector [rad]: ["
            << error.orientation_error.x() << ", "
            << error.orientation_error.y() << ", "
            << error.orientation_error.z() << "]\n";
  std::cout << std::setprecision(4);
  std::cout << "Orientation error norm [rad]:   "
            << error.orientation_error_norm_rad << "\n";
  std::cout << std::setprecision(2);
  std::cout << "Orientation error norm [deg]:   "
            << error.orientation_error_norm_deg << "\n";
  std::cout << "======================================" << std::endl;
}

// Called after control has stopped (no concurrent writer): writes the retained
// ring-buffer rows to a timestamped CSV so return-to-target accuracy can be
// analysed offline.
std::string shellQuote(const std::string &value) {
  std::string quoted = "'";
  for (char c : value) {
    if (c == '\'') {
      quoted += "'\\''";
    } else {
      quoted += c;
    }
  }
  quoted += "'";
  return quoted;
}

std::optional<std::filesystem::path>
writePoseLogCsv(const std::vector<PoseSample> &log, size_t count,
                const std::string &output_dir) {
  if (count == 0) {
    std::cerr << "No pose samples were recorded; skipping pose CSV."
              << std::endl;
    return std::nullopt;
  }
  const size_t retained = std::min(count, log.size());
  const size_t start = count - retained;

  std::time_t now = std::time(nullptr);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
  std::filesystem::path directory(output_dir.empty() ? "." : output_dir);
  std::error_code ec;
  std::filesystem::create_directories(directory, ec);
  if (ec) {
    std::cerr << "Could not create pose log directory '" << directory.string()
              << "': " << ec.message() << std::endl;
    return std::nullopt;
  }
  const std::filesystem::path path =
      directory / (std::string("pose_error_") + stamp + ".csv");

  std::ofstream out(path);
  if (!out) {
    std::cerr << "Could not open " << path.string() << " to save the pose log."
              << std::endl;
    return std::nullopt;
  }
  out << std::fixed << std::setprecision(6);
  out << "t,"
         "target_x,target_y,target_z,"
         "actual_x,actual_y,actual_z,"
         "inner_x,inner_y,inner_z,"
         "position_error_x,position_error_y,position_error_z,position_error_"
         "norm,"
         "orientation_error_x,orientation_error_y,orientation_error_z,"
         "orientation_error_norm_rad,orientation_error_norm_deg,"
         "target_roll_deg,target_pitch_deg,target_yaw_deg,"
         "actual_roll_deg,actual_pitch_deg,actual_yaw_deg,"
         "error_roll_deg,error_pitch_deg,error_yaw_deg,"
         "inner_vx,inner_vy,inner_vz,"
         "filtered_fx,filtered_fy,filtered_fz,"
         "masked_fx,masked_fy,masked_fz\n";
  for (size_t k = start; k < count; ++k) {
    const PoseSample &s = log[k % log.size()];
    out << s.t;
    for (double v : s.target)
      out << "," << v;
    for (double v : s.actual)
      out << "," << v;
    for (double v : s.inner)
      out << "," << v;
    for (double v : s.position_error)
      out << "," << v;
    out << "," << s.position_error_norm;
    for (double v : s.orientation_error)
      out << "," << v;
    out << "," << s.orientation_error_norm_rad << ","
        << s.orientation_error_norm_deg;
    for (double v : s.target_rpy_deg)
      out << "," << v;
    for (double v : s.actual_rpy_deg)
      out << "," << v;
    for (double v : s.error_rpy_deg)
      out << "," << v;
    for (double v : s.inner_velocity)
      out << "," << v;
    for (double v : s.filtered_force)
      out << "," << v;
    for (double v : s.masked_force)
      out << "," << v;
    out << "\n";
  }
  std::cerr << "Recorded " << retained << " pose samples to " << path.string()
            << std::endl;
  return path;
}

void plotPoseLogCsv(const std::filesystem::path &csv_path,
                    const std::string &plot_script,
                    const std::string &output_dir) {
  std::filesystem::path directory(output_dir.empty() ? "." : output_dir);
  std::error_code ec;
  std::filesystem::create_directories(directory, ec);
  if (ec) {
    std::cerr << "Could not create plot output directory '"
              << directory.string() << "': " << ec.message() << std::endl;
    return;
  }

  const std::string csv_stem = csv_path.stem().string();
  const std::string prefix = "pose_error_";
  const std::string suffix = csv_stem.rfind(prefix, 0) == 0
                                 ? csv_stem.substr(prefix.size())
                                 : csv_stem;
  const std::filesystem::path output_path =
      directory / (std::string("ft_sensor_and_pose_error_") + suffix + ".png");

  const std::string command =
      "MPLCONFIGDIR=/tmp python " + shellQuote(plot_script) + " " +
      shellQuote(csv_path.string()) + " -o " + shellQuote(output_path.string());

  std::cerr << "Generating plot from " << csv_path.string() << std::endl;
  const int status = std::system(command.c_str());
  if (status != 0) {
    std::cerr << "Auto plot command failed with status " << status
              << ". Command: " << command << std::endl;
    return;
  }
  std::cerr << "Saved plot to " << output_path.string() << std::endl;
}

// ---- Unfiltered vs low-pass-filtered force comparison logging (opt-in) ----

// One control-cycle snapshot of the external force just before and just after
// the first-order low-pass filter. Kept in a preallocated ring buffer so no
// allocation happens in the real-time callback. Both vectors come from the SAME
// control cycle and are expressed in the explicitly selected compliance frame
// used by the admittance loop.
struct FilterSample {
  double t = 0.0;
  std::array<double, 3> unfiltered{};
  std::array<double, 3> filtered{};
};

// Called after control has stopped (no concurrent writer): creates the
// comparison output directory if needed and writes the retained ring-buffer
// rows to a timestamped CSV. The directory creation and file write happen here,
// never in the real-time callback. Returns the CSV path (with its generated
// timestamp) so the caller can reuse it for the PNG.
std::optional<std::filesystem::path>
writeFilterComparisonCsv(const std::vector<FilterSample> &log, size_t count) {
  if (count == 0) {
    std::cerr << "No filter-comparison samples were recorded; skipping "
                 "comparison CSV."
              << std::endl;
    return std::nullopt;
  }
  const size_t retained = std::min(count, log.size());
  const size_t start = count - retained;

  std::time_t now = std::time(nullptr);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));

  std::filesystem::path directory(kFilterComparisonDir);
  std::error_code ec;
  std::filesystem::create_directories(directory, ec);
  if (ec) {
    std::cerr << "Could not create filter-comparison directory '"
              << directory.string() << "': " << ec.message() << std::endl;
    return std::nullopt;
  }
  const std::filesystem::path path =
      directory / (std::string("wrench_filter_comparison_") + stamp + ".csv");

  std::ofstream out(path);
  if (!out) {
    std::cerr << "Could not open " << path.string()
              << " to save the filter-comparison log." << std::endl;
    return std::nullopt;
  }
  out << std::fixed << std::setprecision(6);
  out << "time,"
         "unfiltered_fx,unfiltered_fy,unfiltered_fz,"
         "filtered_fx,filtered_fy,filtered_fz\n";
  for (size_t k = start; k < count; ++k) {
    const FilterSample &s = log[k % log.size()];
    out << s.t;
    for (double v : s.unfiltered)
      out << "," << v;
    for (double v : s.filtered)
      out << "," << v;
    out << "\n";
  }
  std::cerr << "Recorded " << retained << " filter-comparison samples to "
            << path.string() << std::endl;
  return path;
}

// Renders the four-subplot (Fx, Fy, Fz, |F|) unfiltered-vs-filtered comparison
// PNG next to the CSV. The PNG reuses the CSV's timestamp because its path is
// the CSV path with a .png extension. Runs the Python plot script after control
// has stopped, never from the RT loop.
void plotFilterComparisonCsv(const std::filesystem::path &csv_path,
                             double filter_alpha) {
  std::filesystem::path output_path = csv_path;
  output_path.replace_extension(".png");

  const std::string command = "MPLCONFIGDIR=/tmp python " +
                              shellQuote(kFilterComparisonPlotScript) + " " +
                              shellQuote(csv_path.string()) + " -o " +
                              shellQuote(output_path.string()) + " --alpha " +
                              shellQuote(std::to_string(filter_alpha));

  std::cerr << "Generating filter-comparison plot from " << csv_path.string()
            << std::endl;
  const int status = std::system(command.c_str());
  if (status != 0) {
    std::cerr << "Filter-comparison plot command failed with status " << status
              << ". Command: " << command << std::endl;
    return;
  }
  std::cerr << "Saved filter-comparison plot to " << output_path.string()
            << std::endl;
}

} // namespace

namespace my_controller {

bool runCartesianAdmittanceToTarget(const CartesianAdmittanceConfig &config) {
  // Initialize ROS only if the caller has not already done so, and remember it
  // so we only shut down what we started. rclcpp::ok() also doubles as a stop
  // signal (Ctrl+C) below.
  const bool ros_owned = !rclcpp::ok();
  if (ros_owned) {
    rclcpp::init(0, nullptr);
  }

  // The wrench-topic source needs a spinning node; serial/franka sources do
  // not. A cancellable executor lets us stop our spin without shutting down a
  // caller's ROS.
  const bool uses_topic = config.wrench_source == "topic";
  std::shared_ptr<RealtimeInputs> node;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread ros_thread;

  WrenchDebugData wrench_debug;
  PoseDebugData pose_debug;
  std::atomic<bool> print_wrench{false};
  std::thread wrench_print_thread;
  std::vector<WrenchSample> wrench_history(kWrenchHistorySize);
  size_t wrench_history_count = 0;

  // Pose-error log ring buffer (allocated only when logging is enabled; capped
  // so an infinite run keeps only the most recent samples) plus the settled
  // pose captured for the final summary. Declared before the try so they
  // survive an exception and can be dumped/printed after control stops.
  const size_t pose_log_capacity =
      config.run_time > 0.0
          ? std::min(static_cast<size_t>(config.run_time * 1000.0) + 1000,
                     kPoseLogMaxCapacity)
          : kPoseLogMaxCapacity;
  std::vector<PoseSample> pose_log(config.log_pose_error ? pose_log_capacity
                                                         : 0);
  size_t pose_log_count = 0;

  // Unfiltered-vs-filtered force comparison ring buffer (allocated only when
  // the comparison option is enabled; capped so an unbounded run keeps only the
  // most recent samples). Declared before the try so it survives an exception
  // and can be dumped after control stops. Empty (no allocation) when the
  // option is off, so the RT loop skips recording.
  const size_t filter_log_capacity =
      config.run_time > 0.0
          ? std::min(static_cast<size_t>(config.run_time * 1000.0) + 1000,
                     kFilterLogMaxCapacity)
          : kFilterLogMaxCapacity;
  std::vector<FilterSample> filter_log(
      config.plot_filter_comparison ? filter_log_capacity : 0);
  size_t filter_log_count = 0;

  Eigen::Vector3d target_position = config.target_position_world;
  Eigen::Matrix3d target_rotation =
      config.target_rotation_world.value_or(Eigen::Matrix3d::Identity());
  std::atomic<bool> settled{false};
  bool settle_captured = false;
  Eigen::Vector3d stable_position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d stable_rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d settled_target_position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d settled_target_rotation = Eigen::Matrix3d::Identity();
  // Stored outside the try block because the printer thread is joined after
  // exception handling; it must never retain a reference to the shorter-lived
  // enum inside try.
  std::string compliance_frame_label = config.admittance_frame;
  bool control_returned_normally = false;

  try {
    std::unique_ptr<SerialWrenchReader> serial_reader;
    if (config.wrench_source == "serial") {
      serial_reader = std::make_unique<SerialWrenchReader>(config.serial_port);
      std::cout << "Reading HEX21 wrench directly from serial port '"
                << config.serial_port << "'." << std::endl;
    }
    if (uses_topic) {
      node = std::make_shared<RealtimeInputs>(config.wrench_topic);
      executor.add_node(node);
      ros_thread = std::thread([&executor]() { executor.spin(); });
    }

    franka::Robot robot(config.robot_hostname);
    robot.automaticErrorRecovery();
    if (config.set_payload) {
      robot.setLoad(config.payload_mass, config.payload_center_of_mass,
                    config.payload_inertia);
      std::cout << "Configured libfranka payload: " << config.payload_mass
                << " kg, flange-to-CoM [" << config.payload_center_of_mass[0]
                << ", " << config.payload_center_of_mass[1] << ", "
                << config.payload_center_of_mass[2] << "] m." << std::endl;
    }
    franka::Model model = robot.loadModel();

    // Deliberately keep the collision thresholds configured on the robot. The
    // previous implementation overwrote every threshold with 100, effectively
    // disabling the reflex. Contact-capable thresholds should be selected
    // explicitly for the experiment, not hidden inside this controller
    // implementation.

    for (double mass : config.admittance_mass) {
      if (!std::isfinite(mass) || mass <= 0.0) {
        throw std::invalid_argument(
            "Every admittance mass must be finite and strictly positive.");
      }
    }
    const ComplianceFrame compliance_frame =
        parseComplianceFrame(config.admittance_frame);
    compliance_frame_label = complianceFrameName(compliance_frame);
    const Eigen::Matrix<double, 6, 6> adm_mass_inv =
        diagonalMatrix(config.admittance_mass).inverse();
    const Eigen::Matrix<double, 6, 6> adm_stiffness =
        diagonalMatrix(config.admittance_stiffness);
    const Eigen::Matrix<double, 6, 6> adm_damping =
        diagonalMatrix(config.admittance_damping);
    const Vector6d admittance_mask =
        Eigen::Map<const Vector6d>(config.admittance_mask.data());

    // Fixed sensor->flange rotation from the configured mounting angles (built
    // once). Maps a vector expressed in the sensor frame into the flange (EE)
    // frame; the live EE rotation then carries it the rest of the way to world.
    constexpr double kDeg2Rad = 0.017453292519943295;
    const Eigen::Matrix3d R_flange_sensor =
        (Eigen::AngleAxisd(config.sensor_mount_rpy_deg[2] * kDeg2Rad,
                           Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(config.sensor_mount_rpy_deg[1] * kDeg2Rad,
                           Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(config.sensor_mount_rpy_deg[0] * kDeg2Rad,
                           Eigen::Vector3d::UnitX()))
            .toRotationMatrix();

    // Rigid tool transforms (built once). F_T_TCP is a pure +Z translation to
    // the tip, so the control point is the TCP; S_p_TCP is the sensor-origin ->
    // TCP offset for the moment shift. Orientation of the TCP equals the flange
    // orientation.
    Eigen::Affine3d F_T_TCP = Eigen::Affine3d::Identity();
    F_T_TCP.translation() =
        Eigen::Map<const Eigen::Vector3d>(config.flange_to_tcp.data());
    const Eigen::Vector3d S_p_TCP =
        Eigen::Map<const Eigen::Vector3d>(config.sensor_to_tcp.data());

    // These transforms are read from the first real RobotState and then cached.
    // setEE/setK cannot be changed during an active motion, and avoiding a 4x4
    // inverse in every RT cycle keeps the geometry path deterministic.
    bool geometry_initialized = false;
    Eigen::Affine3d F_T_EE = Eigen::Affine3d::Identity();
    Eigen::Affine3d EE_T_F = Eigen::Affine3d::Identity();
    Eigen::Affine3d EE_T_TCP = Eigen::Affine3d::Identity();
    Eigen::Affine3d TCP_T_EE = Eigen::Affine3d::Identity();
    Eigen::Affine3d EE_T_K = Eigen::Affine3d::Identity();

    bool initialized = false;
    // The provider's initial_* arguments keep the original public contract:
    // measured physical TCP pose at controller start. The static ramp has a
    // separate start pose based on libfranka's last command so its first new
    // command is continuous.
    Eigen::Vector3d initial_position = Eigen::Vector3d::Zero();
    Eigen::Matrix3d initial_rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d nominal_start_position = Eigen::Vector3d::Zero();
    Eigen::Matrix3d nominal_start_rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d ramp_rotation_vector = Eigen::Vector3d::Zero();
    // External wrench drives only this relative compliant offset. Position,
    // rotation and velocity are stored directly in the selected
    // compliance-frame coordinates. Keeping the state in that frame makes M/D/K
    // constant there; only the final offset vector is rotated into O before
    // composition with the nominal O-frame TCP trajectory.
    Eigen::Vector3d adm_position_offset_compliance = Eigen::Vector3d::Zero();
    Eigen::Matrix3d adm_rotation_offset_compliance =
        Eigen::Matrix3d::Identity();
    Vector6d adm_velocity_compliance = Vector6d::Zero();

    // Bias is estimated in the native source representation (sensor/K/O), so
    // samples are not accidentally averaged across a rotating compliance frame.
    // The low-pass state is kept in compliance-frame components and
    // re-expressed whenever that frame rotates.
    Vector6d wrench_bias_source = Vector6d::Zero();
    Vector6d wrench_bias_accumulator_source = Vector6d::Zero();
    Vector6d wrench_filtered_compliance = Vector6d::Zero();
    Eigen::Matrix3d previous_compliance_rotation = Eigen::Matrix3d::Identity();
    bool wrench_filter_frame_initialized = false;
    int wrench_bias_count = 0;
    double elapsed_time = 0.0;
    std::atomic<bool> control_finished{false};

    // Settling-detector working state (RT-only; not needed after control
    // stops). A dynamic target must first stop changing; then the measured TCP
    // must remain close to the full compliant command. This avoids declaring
    // "settled" while a provider is still moving.
    Eigen::Vector3d previous_desired_position = Eigen::Vector3d::Zero();
    Eigen::Matrix3d previous_desired_rotation = Eigen::Matrix3d::Identity();
    bool previous_desired_valid = false;
    double desired_stationary_time = 0.0;
    double settle_time = 0.0;

    auto cartesian_admittance_callback =
        [&](const franka::RobotState &state,
            franka::Duration period) -> franka::CartesianPose {
      const double dt = period.toSec();
      const Eigen::Affine3d O_T_EE = arrayToTransform(state.O_T_EE);

      if (!geometry_initialized) {
        // libfranka reports O_T_EE, not O_T_F. Recover the physical flange,
        // then append the user-configured physical flange->TCP transform:
        //   O_T_TCP = O_T_EE * inv(F_T_EE) * F_T_TCP.
        F_T_EE = arrayToTransform(state.F_T_EE);
        EE_T_F = F_T_EE.inverse();
        EE_T_TCP = EE_T_F * F_T_TCP;
        TCP_T_EE = EE_T_TCP.inverse();
        EE_T_K = arrayToTransform(state.EE_T_K);
        geometry_initialized = true;
      }

      const Eigen::Affine3d O_T_F = O_T_EE * EE_T_F;
      const Eigen::Affine3d O_T_TCP = O_T_EE * EE_T_TCP;
      const Eigen::Affine3d O_T_K = O_T_EE * EE_T_K;
      const Eigen::Vector3d actual_position = O_T_TCP.translation();
      const Eigen::Matrix3d actual_rotation = O_T_TCP.linear(); // R_O_TCP

      if (!initialized) {
        // Start the nominal trajectory at libfranka's last commanded EE pose,
        // converted to this controller's physical TCP. This keeps the first
        // Cartesian command continuous even when measured pose and last command
        // differ slightly.
        const Eigen::Affine3d O_T_EE_commanded =
            arrayToTransform(state.O_T_EE_c);
        const Eigen::Affine3d O_T_TCP_commanded = O_T_EE_commanded * EE_T_TCP;
        initial_position = actual_position;
        initial_rotation = actual_rotation;
        nominal_start_position = O_T_TCP_commanded.translation();
        nominal_start_rotation = O_T_TCP_commanded.linear();
        if (config.hold_initial_tcp_pose) {
          target_position = initial_position;
          target_rotation = initial_rotation;
        } else {
          target_rotation =
              config.target_rotation_world.value_or(initial_rotation);
        }
        adm_position_offset_compliance.setZero();
        adm_rotation_offset_compliance.setIdentity();
        adm_velocity_compliance.setZero();
        ramp_rotation_vector =
            rotationLog(target_rotation * nominal_start_rotation.transpose());
        initialized = true;
      }

      Eigen::Vector3d desired_position = target_position;
      Eigen::Matrix3d desired_rotation = target_rotation;
      if (config.target_provider) {
        config.target_provider(elapsed_time, initial_position, initial_rotation,
                               desired_position, desired_rotation);
      } else {
        // Minimum-jerk ramp of the desired TCP pose from the start pose to the
        // target, then hold. Prevents a large initial admittance error / sudden
        // command.
        const double ramp =
            minimumJerkScale(elapsed_time, config.target_ramp_time);
        desired_position = nominal_start_position +
                           ramp * (target_position - nominal_start_position);
        desired_rotation = integrateRotation(nominal_start_rotation,
                                             ramp * ramp_rotation_vector);
      }

      // Select one compliance basis C. The nominal trajectory itself remains an
      // absolute physical TCP pose in O/world for every mode.
      Eigen::Matrix3d R_O_C = Eigen::Matrix3d::Identity();
      switch (compliance_frame) {
      case ComplianceFrame::kActualTcp:
        R_O_C = actual_rotation;
        break;
      case ComplianceFrame::kNominalTcp:
        R_O_C = desired_rotation;
        break;
      case ComplianceFrame::kWorld:
        break;
      }
      const Eigen::Matrix3d R_C_O = R_O_C.transpose();

      // Read each source in its native representation. Bias is estimated here,
      // before any moving-frame transform, so samples never mix different
      // compliance-frame bases.
      Vector6d raw_wrench_source = Vector6d::Zero();
      if (config.wrench_source == "franka") {
        raw_wrench_source = frankaWrenchSource(state, config.wrench_frame);
      } else if (config.wrench_source == "serial") {
        raw_wrench_source = serial_reader->wrench();
      } else {
        raw_wrench_source = node->wrench();
      }
      raw_wrench_source *= config.wrench_sign;

      const bool has_wrench =
          config.wrench_source == "franka" ||
          (config.wrench_source == "serial" && serial_reader->hasWrench()) ||
          (config.wrench_source == "topic" && node->hasWrench());
      if (has_wrench && wrench_bias_count < kWrenchBiasSamples) {
        wrench_bias_accumulator_source += raw_wrench_source;
        ++wrench_bias_count;
        wrench_bias_source = wrench_bias_accumulator_source /
                             static_cast<double>(wrench_bias_count);
      }

      const Vector6d wrench_source = raw_wrench_source - wrench_bias_source;

      // Convert a native source wrench into O/world axes, referenced at the
      // physical TCP. Franka's wrench is referenced at stiffness origin K. A
      // local serial/topic wrench is referenced at sensor origin S. A
      // world-frame serial/topic wrench is assumed to have already been shifted
      // to the physical TCP by its publisher.
      const auto sourceToWorldAtTcp = [&](const Vector6d &source_wrench) {
        Vector6d result = Vector6d::Zero();
        if (config.wrench_source == "franka") {
          if (config.wrench_frame == "local") {
            result.head<3>() = O_T_K.linear() * source_wrench.head<3>();
            result.tail<3>() = O_T_K.linear() * source_wrench.tail<3>();
          } else {
            result = source_wrench;
          }
          const Eigen::Vector3d r_O_K_TCP =
              O_T_TCP.translation() - O_T_K.translation();
          result.tail<3>() -= r_O_K_TCP.cross(result.head<3>());
        } else if (config.wrench_frame == "local") {
          const Eigen::Matrix3d R_O_S = O_T_F.linear() * R_flange_sensor;
          result.head<3>() = R_O_S * source_wrench.head<3>();
          result.tail<3>() = R_O_S * source_wrench.tail<3>();
          const Eigen::Vector3d r_O_S_TCP = R_O_S * S_p_TCP;
          result.tail<3>() -= r_O_S_TCP.cross(result.head<3>());
        } else {
          result = source_wrench;
        }
        return result;
      };

      const Vector6d raw_wrench_world_at_tcp =
          sourceToWorldAtTcp(raw_wrench_source);
      const Vector6d wrench_world_at_tcp = sourceToWorldAtTcp(wrench_source);
      Vector6d raw_wrench_compliance;
      raw_wrench_compliance.head<3>() =
          R_C_O * raw_wrench_world_at_tcp.head<3>();
      raw_wrench_compliance.tail<3>() =
          R_C_O * raw_wrench_world_at_tcp.tail<3>();
      Vector6d wrench_compliance;
      wrench_compliance.head<3>() = R_C_O * wrench_world_at_tcp.head<3>();
      wrench_compliance.tail<3>() = R_C_O * wrench_world_at_tcp.tail<3>();

      // Disable masked-out compliance axes before
      // deadband/saturation/filtering, so a 0.0 axis contributes no admittance
      // force regardless of the measured wrench.
      const Vector6d wrench_masked_compliance =
          wrench_compliance.cwiseProduct(admittance_mask);
      Vector6d wrench_deadbanded_compliance = wrench_masked_compliance;
      Eigen::Vector3d limited_force = wrench_deadbanded_compliance.head<3>();
      applyVectorNormDeadband(limited_force, config.force_deadband);
      limitVectorNorm(limited_force, config.max_external_force);
      wrench_deadbanded_compliance.head<3>() = limited_force;
      Eigen::Vector3d limited_torque = wrench_deadbanded_compliance.tail<3>();
      applyVectorNormDeadband(limited_torque, config.torque_deadband);
      limitVectorNorm(limited_torque, config.max_external_torque);
      wrench_deadbanded_compliance.tail<3>() = limited_torque;

      // The old filter output represents a physical wrench in last cycle's
      // basis. Rotate those stored components into this cycle's C basis before
      // applying the IIR update.
      if (wrench_filter_frame_initialized) {
        const Eigen::Matrix3d R_C_current_C_previous =
            R_C_O * previous_compliance_rotation;
        const Eigen::Vector3d previous_filtered_force =
            wrench_filtered_compliance.head<3>();
        const Eigen::Vector3d previous_filtered_torque =
            wrench_filtered_compliance.tail<3>();
        wrench_filtered_compliance.head<3>() =
            R_C_current_C_previous * previous_filtered_force;
        wrench_filtered_compliance.tail<3>() =
            R_C_current_C_previous * previous_filtered_torque;
      } else {
        wrench_filter_frame_initialized = true;
      }
      previous_compliance_rotation = R_O_C;
      wrench_filtered_compliance =
          config.wrench_filter_alpha * wrench_deadbanded_compliance +
          (1.0 - config.wrench_filter_alpha) * wrench_filtered_compliance;

      storeVector(wrench_debug.raw, raw_wrench_compliance);
      storeVector(wrench_debug.bias_removed, wrench_compliance);
      storeVector(wrench_debug.masked, wrench_masked_compliance);
      storeVector(wrench_debug.deadbanded, wrench_deadbanded_compliance);
      storeVector(wrench_debug.filtered, wrench_filtered_compliance);
      wrench_debug.bias_count.store(wrench_bias_count,
                                    std::memory_order_relaxed);
      wrench_debug.has_sample.store(true, std::memory_order_release);

      // Append to the ring buffer (preallocated, no allocation in the RT loop).
      elapsed_time += dt;
      WrenchSample &history_sample =
          wrench_history[wrench_history_count % kWrenchHistorySize];
      history_sample.t = elapsed_time;
      history_sample.bias_count = wrench_bias_count;
      history_sample.raw = toArray(raw_wrench_compliance);
      history_sample.bias_removed = toArray(wrench_compliance);
      history_sample.masked = toArray(wrench_masked_compliance);
      history_sample.deadbanded = toArray(wrench_deadbanded_compliance);
      history_sample.filtered = toArray(wrench_filtered_compliance);
      ++wrench_history_count;

      // M*x_ddot + D*x_dot + K*x = wrench_ext. Every term below is represented
      // in the selected C axes. Existing compliant state is intentionally
      // attached to C: in actual_tcp/nominal_tcp modes its world direction
      // follows that moving frame.
      Vector6d adm_offset_compliance;
      adm_offset_compliance.head<3>() = adm_position_offset_compliance;
      adm_offset_compliance.tail<3>() =
          rotationLog(adm_rotation_offset_compliance);
      for (int i = 0; i < 6; ++i) {
        if (admittance_mask(i) == 0.0) {
          adm_offset_compliance(i) = 0.0;
          adm_velocity_compliance(i) = 0.0;
        }
      }
      adm_position_offset_compliance = adm_offset_compliance.head<3>();
      adm_rotation_offset_compliance = integrateRotation(
          Eigen::Matrix3d::Identity(), adm_offset_compliance.tail<3>());

      const Vector6d adm_force_compliance =
          wrench_filtered_compliance - adm_damping * adm_velocity_compliance -
          adm_stiffness * adm_offset_compliance;
      Vector6d adm_acceleration_compliance =
          adm_mass_inv * adm_force_compliance;
      for (int i = 0; i < 6; ++i) {
        if (admittance_mask(i) == 0.0) {
          adm_acceleration_compliance(i) = 0.0;
        }
      }
      adm_velocity_compliance += adm_acceleration_compliance * dt;
      Eigen::Vector3d linear_velocity = adm_velocity_compliance.head<3>();
      Eigen::Vector3d angular_velocity = adm_velocity_compliance.tail<3>();
      limitVectorNorm(linear_velocity, config.max_linear_speed);
      limitVectorNorm(angular_velocity, config.max_angular_speed);
      adm_velocity_compliance.head<3>() = linear_velocity;
      adm_velocity_compliance.tail<3>() = angular_velocity;

      adm_position_offset_compliance += adm_velocity_compliance.head<3>() * dt;
      adm_rotation_offset_compliance =
          integrateRotation(adm_rotation_offset_compliance,
                            adm_velocity_compliance.tail<3>() * dt);

      if (adm_position_offset_compliance.norm() >
          config.max_translation_offset) {
        const Eigen::Vector3d boundary_normal =
            adm_position_offset_compliance.normalized();
        adm_position_offset_compliance =
            boundary_normal * config.max_translation_offset;
        const double outward_speed =
            boundary_normal.dot(adm_velocity_compliance.head<3>());
        if (outward_speed > 0.0) {
          adm_velocity_compliance.head<3>() -= outward_speed * boundary_normal;
        }
      }

      Eigen::Vector3d rotation_offset_compliance =
          rotationLog(adm_rotation_offset_compliance);
      if (rotation_offset_compliance.norm() > config.max_rotation_offset) {
        const Eigen::Vector3d boundary_normal =
            rotation_offset_compliance.normalized();
        rotation_offset_compliance =
            boundary_normal * config.max_rotation_offset;
        const double outward_speed =
            boundary_normal.dot(adm_velocity_compliance.tail<3>());
        if (outward_speed > 0.0) {
          adm_velocity_compliance.tail<3>() -= outward_speed * boundary_normal;
        }
        adm_rotation_offset_compliance = integrateRotation(
            Eigen::Matrix3d::Identity(), rotation_offset_compliance);
      }

      // Re-pin disabled axes after integration, then rebuild the rotation state
      // from its masked logarithmic coordinates.
      for (int i = 0; i < 3; ++i) {
        if (admittance_mask(i) == 0.0) {
          adm_velocity_compliance(i) = 0.0;
          adm_position_offset_compliance(i) = 0.0;
        }
        if (admittance_mask(3 + i) == 0.0) {
          rotation_offset_compliance(i) = 0.0;
          adm_velocity_compliance(3 + i) = 0.0;
        }
      }
      adm_rotation_offset_compliance = integrateRotation(
          Eigen::Matrix3d::Identity(), rotation_offset_compliance);

      // Convert only the relative compliance output C->O, then compose it with
      // the nominal absolute TCP trajectory. Finally convert physical TCP back
      // to libfranka's EE frame.
      const Eigen::Vector3d command_position =
          desired_position + R_O_C * adm_position_offset_compliance;
      const Eigen::Matrix3d command_rotation = integrateRotation(
          desired_rotation, R_O_C * rotation_offset_compliance);
      Eigen::Affine3d O_T_TCP_cmd = Eigen::Affine3d::Identity();
      O_T_TCP_cmd.linear() = command_rotation;
      O_T_TCP_cmd.translation() = command_position;
      const Eigen::Affine3d O_T_EE_cmd = O_T_TCP_cmd * TCP_T_EE;

      if (config.tcp_pose_observer) {
        config.tcp_pose_observer(actual_position, actual_rotation,
                                 command_position, command_rotation);
      }
      if (config.joint_position_observer) {
        config.joint_position_observer(state.q);
      }

      franka::CartesianPose pose(
          transformToArray(O_T_EE_cmd.translation(), O_T_EE_cmd.linear()));

      // --- Pose-error monitoring against the target, using the measured TCP
      // pose ---
      const PoseError pose_error = computePoseError(
          desired_position, desired_rotation, actual_position, actual_rotation);
      storeVector3(pose_debug.actual, actual_position);
      storeVector3(pose_debug.position_error, pose_error.position_error);
      pose_debug.position_error_norm.store(pose_error.position_error_norm,
                                           std::memory_order_relaxed);
      storeVector3(pose_debug.orientation_error, pose_error.orientation_error);
      pose_debug.orientation_error_norm_rad.store(
          pose_error.orientation_error_norm_rad, std::memory_order_relaxed);
      pose_debug.orientation_error_norm_deg.store(
          pose_error.orientation_error_norm_deg, std::memory_order_relaxed);
      storeVector3(pose_debug.orientation_error_rpy_deg,
                   pose_error.orientation_error_rpy * kRadToDeg);
      storeVector3(pose_debug.actual_rpy_deg,
                   pose_error.actual_rpy * kRadToDeg);
      storeVector3(pose_debug.target_rpy_deg,
                   pose_error.target_rpy * kRadToDeg);
      pose_debug.has_sample.store(true, std::memory_order_release);

      // --- Settling detector. The provider must be effectively stationary at
      // the strict target-speed epsilons above. Then measured physical TCP must
      // track the compliant command and compliance velocity must stay small for
      // settle_window seconds. ---
      bool desired_stationary = false;
      if (previous_desired_valid && dt > 0.0) {
        const double desired_linear_speed =
            (desired_position - previous_desired_position).norm() / dt;
        const double desired_angular_speed =
            rotationLog(desired_rotation *
                        previous_desired_rotation.transpose())
                .norm() /
            dt;
        desired_stationary =
            desired_linear_speed < kTargetStationaryLinearSpeed &&
            desired_angular_speed < kTargetStationaryAngularSpeed;
      }
      if (desired_stationary) {
        desired_stationary_time += dt;
      } else {
        desired_stationary_time = 0.0;
      }
      previous_desired_position = desired_position;
      previous_desired_rotation = desired_rotation;
      previous_desired_valid = true;

      bool settled_now = settle_captured;
      if (!settle_captured) {
        const double command_position_error =
            (command_position - actual_position).norm();
        const double command_rotation_error =
            rotationLog(command_rotation * actual_rotation.transpose()).norm();
        const bool static_ramp_finished =
            config.target_provider || elapsed_time >= config.target_ramp_time;
        const bool within =
            static_ramp_finished && desired_stationary_time > 0.0 &&
            command_position_error < config.settle_position_threshold &&
            command_rotation_error < config.settle_rotation_threshold &&
            adm_velocity_compliance.head<3>().norm() <
                config.settle_velocity_threshold &&
            adm_velocity_compliance.tail<3>().norm() <
                config.settle_velocity_threshold;
        if (within) {
          settle_time += dt;
          if (settle_time >= config.settle_window) {
            settle_captured = true;
            settled_now = true;
            stable_position = actual_position;
            stable_rotation = actual_rotation;
            settled_target_position = desired_position;
            settled_target_rotation = desired_rotation;
            settled.store(true, std::memory_order_release);
          }
        } else {
          settle_time = 0.0;
        }
      }

      // --- Pose-error CSV ring buffer (preallocated; only when logging is
      // enabled) ---
      if (config.log_pose_error) {
        PoseSample &sample = pose_log[pose_log_count % pose_log.size()];
        sample.t = elapsed_time;
        sample.target = toArray3(desired_position);
        sample.actual = toArray3(actual_position);
        sample.inner = toArray3(command_position);
        sample.position_error = toArray3(pose_error.position_error);
        sample.position_error_norm = pose_error.position_error_norm;
        sample.orientation_error = toArray3(pose_error.orientation_error);
        sample.orientation_error_norm_rad =
            pose_error.orientation_error_norm_rad;
        sample.orientation_error_norm_deg =
            pose_error.orientation_error_norm_deg;
        sample.target_rpy_deg = toArray3(pose_error.target_rpy * kRadToDeg);
        sample.actual_rpy_deg = toArray3(pose_error.actual_rpy * kRadToDeg);
        sample.error_rpy_deg =
            toArray3(pose_error.orientation_error_rpy * kRadToDeg);
        sample.inner_velocity = toArray3(adm_velocity_compliance.head<3>());
        sample.filtered_force = toArray3(wrench_filtered_compliance.head<3>());
        sample.masked_force = toArray3(wrench_masked_compliance.head<3>());
        ++pose_log_count;
      }

      // --- Unfiltered vs filtered force comparison ring buffer (preallocated;
      // only when the comparison option is enabled). Records
      // selected-compliance-frame force immediately before the low-pass filter
      // and at the filter output from the SAME control cycle. Comparison-only:
      // the admittance loop above already consumed the filtered wrench, so this
      // does not change control behavior. ---
      if (config.plot_filter_comparison) {
        FilterSample &filter_sample =
            filter_log[filter_log_count % filter_log.size()];
        filter_sample.t = elapsed_time;
        filter_sample.unfiltered =
            toArray3(wrench_deadbanded_compliance.head<3>());
        filter_sample.filtered = toArray3(wrench_filtered_compliance.head<3>());
        ++filter_log_count;
      }

      const bool stop =
          (config.stop_requested &&
           config.stop_requested->load(std::memory_order_relaxed)) ||
          (config.run_time >= 0.0 && elapsed_time >= config.run_time) ||
          (config.stop_on_settled && settled_now) || !rclcpp::ok();
      if (stop) {
        control_finished.store(true, std::memory_order_release);
        return franka::MotionFinished(pose);
      }
      return pose;
    };

    auto joint_impedance_callback =
        [&](const franka::RobotState &state,
            franka::Duration /*period*/) -> franka::Torques {
      const std::array<double, 7> coriolis = model.coriolis(state);

      std::array<double, 7> tau{};
      for (size_t i = 0; i < tau.size(); ++i) {
        tau[i] = config.joint_stiffness[i] * (state.q_d[i] - state.q[i]) -
                 config.joint_damping[i] * state.dq[i] + coriolis[i];
      }

      const std::array<double, 7> tau_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau, state.tau_J_d);
      franka::Torques command(tau_rate_limited);
      if (control_finished.load(std::memory_order_acquire) || !rclcpp::ok()) {
        return franka::MotionFinished(command);
      }
      return command;
    };

    std::cout << std::fixed << std::setprecision(3);
    if (config.target_provider) {
      std::cout
          << "Cartesian admittance trajectory target provider active, run-time "
          << config.run_time << " s.\n";
    } else if (config.hold_initial_tcp_pose) {
      std::cout << "Cartesian admittance will capture and hold the physical "
                   "TCP pose measured at controller startup, ramp "
                << config.target_ramp_time << " s.\n";
    } else {
      std::cout << "Cartesian admittance -> target TCP position (world) = ["
                << config.target_position_world.x() << ", "
                << config.target_position_world.y() << ", "
                << config.target_position_world.z() << "] m, ramp "
                << config.target_ramp_time << " s.\n";
    }
    std::cout << "Wrench source: " << config.wrench_source
              << ", source frame: " << config.wrench_frame
              << ", sign: " << config.wrench_sign
              << ". Admittance/compliance frame C: " << compliance_frame_label
              << ". Debug wrench values use C axes and are referenced at "
                 "physical TCP. "
                 "Keep the robot unloaded during the first "
              << kWrenchBiasSamples
              << " valid wrench samples. Starting control." << std::endl;

    // Live terminal printing is independent of CSV logging and of the control
    // loop: it is handled here in a non-real-time thread, gated per line by the
    // config flags. The force sensor, wrench pipeline and admittance control
    // always run regardless, so disabling a print only silences the terminal.
    const bool live_printing =
        config.print_pose_error || config.print_wrench_debug;
    const auto print_period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(std::max(config.print_period, 0.01)));
    if (live_printing) {
      print_wrench.store(true, std::memory_order_release);
      wrench_print_thread = std::thread([&, print_period]() {
        std::cout << std::fixed << std::setprecision(3);
        while (print_wrench.load(std::memory_order_acquire) && rclcpp::ok()) {
          if (config.print_wrench_debug &&
              wrench_debug.has_sample.load(std::memory_order_acquire)) {
            const Vector6d raw_wrench = loadVector(wrench_debug.raw);
            const Vector6d bias_removed_wrench =
                loadVector(wrench_debug.bias_removed);
            const Vector6d masked_wrench = loadVector(wrench_debug.masked);
            const Vector6d deadbanded_wrench =
                loadVector(wrench_debug.deadbanded);
            const Vector6d filtered_wrench = loadVector(wrench_debug.filtered);
            const int bias_count =
                wrench_debug.bias_count.load(std::memory_order_relaxed);

            std::cout << "[ft_sensor " << compliance_frame_label
                      << " axes @ physical TCP] ";
            printExternalForceLine(bias_removed_wrench);
            std::cout << " | bias " << bias_count << "/" << kWrenchBiasSamples
                      << " | ";
            printWrenchLine("raw", raw_wrench);
            std::cout << " | ";
            printWrenchLine("bias_removed", bias_removed_wrench);
            std::cout << " | ";
            printWrenchLine("masked", masked_wrench);
            std::cout << " | ";
            printWrenchLine("deadbanded", deadbanded_wrench);
            std::cout << " | ";
            printWrenchLine("filtered", filtered_wrench);
            std::cout << std::endl;
          }
          if (config.print_pose_error &&
              pose_debug.has_sample.load(std::memory_order_acquire)) {
            const double error_px =
                pose_debug.position_error[0].load(std::memory_order_relaxed);
            const double error_py =
                pose_debug.position_error[1].load(std::memory_order_relaxed);
            const double error_pz =
                pose_debug.position_error[2].load(std::memory_order_relaxed);
            const double error_pnorm =
                pose_debug.position_error_norm.load(std::memory_order_relaxed);
            const double error_deg = pose_debug.orientation_error_norm_deg.load(
                std::memory_order_relaxed);
            const double err_roll =
                pose_debug.orientation_error_rpy_deg[0].load(
                    std::memory_order_relaxed);
            const double err_pitch =
                pose_debug.orientation_error_rpy_deg[1].load(
                    std::memory_order_relaxed);
            const double err_yaw = pose_debug.orientation_error_rpy_deg[2].load(
                std::memory_order_relaxed);
            const double act_roll =
                pose_debug.actual_rpy_deg[0].load(std::memory_order_relaxed);
            const double act_pitch =
                pose_debug.actual_rpy_deg[1].load(std::memory_order_relaxed);
            const double act_yaw =
                pose_debug.actual_rpy_deg[2].load(std::memory_order_relaxed);
            const double tgt_roll =
                pose_debug.target_rpy_deg[0].load(std::memory_order_relaxed);
            const double tgt_pitch =
                pose_debug.target_rpy_deg[1].load(std::memory_order_relaxed);
            const double tgt_yaw =
                pose_debug.target_rpy_deg[2].load(std::memory_order_relaxed);
            std::cout << "[pose] e_p[m]=[" << error_px << ", " << error_py
                      << ", " << error_pz << "] |e_p|=" << error_pnorm
                      << " m | e_RPY[deg]=[" << err_roll << ", " << err_pitch
                      << ", " << err_yaw << "] | |e_R|=" << error_deg << " deg"
                      << " | actual_RPY[deg]=[" << act_roll << ", " << act_pitch
                      << ", " << act_yaw << "] target_RPY[deg]=[" << tgt_roll
                      << ", " << tgt_pitch << ", " << tgt_yaw << "]"
                      << (settled.load(std::memory_order_acquire) ? " | SETTLED"
                                                                  : "")
                      << std::endl;
          }
          std::this_thread::sleep_for(print_period);
        }
      });
    }

    robot.control(joint_impedance_callback, cartesian_admittance_callback);
    control_returned_normally = true;
  } catch (const franka::ControlException &ex) {
    // The reflex reason is already in ex.what(); the flags below pinpoint it.
    std::cerr << "libfranka control error: " << ex.what() << std::endl;
    // Scan the whole log (newest first) for the state that actually latched the
    // error flags -- the very last record is often already cleared.
    bool found_errors = false;
    for (auto it = ex.log.rbegin(); it != ex.log.rend(); ++it) {
      const franka::RobotState &s = it->state;
      if (!s.last_motion_errors && !s.current_errors) {
        continue;
      }
      std::cerr << "Trigger (last_motion_errors): " << s.last_motion_errors
                << std::endl;
      std::cerr << "Still active (current_errors): " << s.current_errors
                << std::endl;
      std::cerr << std::fixed << std::setprecision(2)
                << "Robot-estimated external wrench O_F_ext_hat_K = ["
                << s.O_F_ext_hat_K[0] << ", " << s.O_F_ext_hat_K[1] << ", "
                << s.O_F_ext_hat_K[2] << " N | " << s.O_F_ext_hat_K[3] << ", "
                << s.O_F_ext_hat_K[4] << ", " << s.O_F_ext_hat_K[5] << " Nm]"
                << std::endl;
      found_errors = true;
      break;
    }
    if (!found_errors) {
      std::cerr << "No error flags were set across the " << ex.log.size()
                << " logged state(s); the reason is the message above."
                << std::endl;
    }
    dumpWrenchHistory(wrench_history, wrench_history_count, ex.what());
  } catch (const franka::Exception &ex) {
    std::cerr << "libfranka error: " << ex.what() << std::endl;
    dumpWrenchHistory(wrench_history, wrench_history_count, ex.what());
  } catch (const std::exception &ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    dumpWrenchHistory(wrench_history, wrench_history_count, ex.what());
  }

  print_wrench.store(false, std::memory_order_release);
  if (wrench_print_thread.joinable()) {
    wrench_print_thread.join();
  }

  // Settled-pose error summary and pose CSV are emitted here, after control has
  // stopped, so nothing is printed or written from the real-time callback.
  if (settle_captured) {
    const PoseError final_error =
        computePoseError(settled_target_position, settled_target_rotation,
                         stable_position, stable_rotation);
    printSettledSummary(settled_target_position, stable_position, final_error);
  } else {
    std::cout << "Controller stopped before a settled state was detected "
                 "(no settled pose summary)."
              << std::endl;
  }
  std::optional<std::filesystem::path> pose_log_path;
  if (config.log_pose_error) {
    pose_log_path =
        writePoseLogCsv(pose_log, pose_log_count, config.pose_log_dir);
  } else if (config.auto_plot) {
    std::cerr << "Warning: --auto-plot true requires --log-pose-error true. "
                 "No pose-error CSV was written, so no plot will be generated."
              << std::endl;
  }
  if (config.auto_plot && pose_log_path.has_value()) {
    plotPoseLogCsv(*pose_log_path, config.plot_script, config.plot_output_dir);
  }

  // Unfiltered-vs-filtered comparison CSV + PNG, emitted here after control has
  // completely stopped (normal stop or a caught control exception, as long as
  // samples were recorded). Nothing here runs inside the real-time callback.
  if (config.plot_filter_comparison) {
    const std::optional<std::filesystem::path> filter_csv_path =
        writeFilterComparisonCsv(filter_log, filter_log_count);
    if (filter_csv_path.has_value()) {
      plotFilterComparisonCsv(*filter_csv_path, config.wrench_filter_alpha);
    }
  }

  if (uses_topic) {
    executor.cancel();
    if (ros_thread.joinable()) {
      ros_thread.join();
    }
  }
  if (ros_owned) {
    rclcpp::shutdown();
  }
  return control_returned_normally;
}

} // namespace my_controller
