// Small trajectory runner for the tuned Cartesian admittance controller.
//
// It reuses cartesian_admittance.yaml for the compliant controller gains, wrench source,
// limits, logging, and soft/hard profiles, then supplies a dynamic TCP target:
// forward, back to start, backward, back to start, right, back to start, left,
// back to start, and a small circle in the world XY plane.
//
// Build:
//   pixi run -e jazzy-crisp colcon build --packages-select my_controller --symlink-install
//
// Run:
//   pixi run -e jazzy-crisp ros2 run my_controller cartesian_admittance_trajectory 10.90.90.10
//   pixi run -e jazzy-crisp ros2 run my_controller cartesian_admittance_trajectory --help
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

#include <Eigen/Dense>

#include <franka/exception.h>
#include <franka/robot.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include "my_controller/cartesian_admittance.hpp"

namespace {

using my_controller::CartesianAdmittanceConfig;

constexpr const char * kDefaultConfigPath =
  "/home/wenbin/github_repo/crisp_franka/src/my_controller/scripts/cartesian_admittance.yaml";
constexpr double kPi = 3.14159265358979323846;

struct TrajectoryParams {
  double step = 0.03;           // m, line move distance for forward/back/right/left
  double circle_radius = 0.02;  // m
  double segment_time = 2.0;    // s, each straight segment
  double hold_time = 1.0;       // s, before and after the trajectory
  double circle_time = 8.0;     // s, total circle duration
  int circle_revolutions = 1;
};

struct PoseTrajectorySample {
  double t = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
};

struct CsvTrajectory {
  std::string path;
  std::vector<PoseTrajectorySample> samples;
  bool relative = true;
};

double asDouble(const YAML::Node & node, const std::string & key) {
  try {
    return node.as<double>();
  } catch (const YAML::Exception &) {
    throw std::invalid_argument(
      "config key '" + key + "': expected a number, got '" + node.as<std::string>("?") + "'.");
  }
}

bool asBool(const YAML::Node & node, const std::string & key) {
  const std::string raw = node.as<std::string>("?");
  std::string lower;
  lower.reserve(raw.size());
  for (char c : raw) {
    lower += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  if (lower == "true" || lower == "1" || lower == "yes" || lower == "on") {
    return true;
  }
  if (lower == "false" || lower == "0" || lower == "no" || lower == "off") {
    return false;
  }
  throw std::invalid_argument(
    "config key '" + key + "': expected a boolean (true/false), got '" + raw + "'.");
}

std::array<double, 3> asVector3(const YAML::Node & node, const std::string & key) {
  if (!node.IsSequence()) {
    throw std::invalid_argument("config key '" + key + "': expected a list of 3 numbers.");
  }
  std::vector<double> values;
  try {
    values = node.as<std::vector<double>>();
  } catch (const YAML::Exception &) {
    throw std::invalid_argument("config key '" + key + "': expected 3 numbers.");
  }
  if (values.size() != 3) {
    throw std::invalid_argument(
      "config key '" + key + "': expected exactly 3 numbers, got " +
      std::to_string(values.size()) + ".");
  }
  std::array<double, 3> out{};
  std::copy(values.begin(), values.end(), out.begin());
  return out;
}

std::array<double, 6> asVector6(const YAML::Node & node, const std::string & key) {
  if (!node.IsSequence()) {
    throw std::invalid_argument("config key '" + key + "': expected a list of 6 numbers.");
  }
  std::vector<double> values;
  try {
    values = node.as<std::vector<double>>();
  } catch (const YAML::Exception &) {
    throw std::invalid_argument("config key '" + key + "': expected 6 numbers.");
  }
  if (values.size() != 6) {
    throw std::invalid_argument(
      "config key '" + key + "': expected exactly 6 numbers, got " +
      std::to_string(values.size()) + ".");
  }
  std::array<double, 6> out{};
  std::copy(values.begin(), values.end(), out.begin());
  return out;
}

std::array<double, 7> asVector7(const YAML::Node & node, const std::string & key) {
  if (!node.IsSequence()) {
    throw std::invalid_argument("config key '" + key + "': expected a list of 7 numbers.");
  }
  std::vector<double> values;
  try {
    values = node.as<std::vector<double>>();
  } catch (const YAML::Exception &) {
    throw std::invalid_argument("config key '" + key + "': expected 7 numbers.");
  }
  if (values.size() != 7) {
    throw std::invalid_argument(
      "config key '" + key + "': expected exactly 7 numbers, got " +
      std::to_string(values.size()) + ".");
  }
  std::array<double, 7> out{};
  std::copy(values.begin(), values.end(), out.begin());
  return out;
}

std::array<double, 9> asVector9(const YAML::Node & node, const std::string & key) {
  if (!node.IsSequence()) {
    throw std::invalid_argument("config key '" + key + "': expected a list of 9 numbers.");
  }
  std::vector<double> values;
  try {
    values = node.as<std::vector<double>>();
  } catch (const YAML::Exception &) {
    throw std::invalid_argument("config key '" + key + "': expected 9 numbers.");
  }
  if (values.size() != 9) {
    throw std::invalid_argument(
      "config key '" + key + "': expected exactly 9 numbers, got " +
      std::to_string(values.size()) + ".");
  }
  std::array<double, 9> out{};
  std::copy(values.begin(), values.end(), out.begin());
  return out;
}

void applyAdmittance(
  const YAML::Node & node, CartesianAdmittanceConfig & config, const std::string & ctx) {
  if (const YAML::Node v = node["mask"]) config.admittance_mask = asVector6(v, ctx + ".mask");
  if (const YAML::Node v = node["mass"]) config.admittance_mass = asVector6(v, ctx + ".mass");
  if (const YAML::Node v = node["stiffness"]) {
    config.admittance_stiffness = asVector6(v, ctx + ".stiffness");
  }
  if (const YAML::Node v = node["damping"]) {
    config.admittance_damping = asVector6(v, ctx + ".damping");
  }
}

std::string loadConfigFile(
  const std::string & path,
  const std::optional<std::string> & requested_profile,
  CartesianAdmittanceConfig & config) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::BadFile &) {
    throw std::invalid_argument("Could not open config file '" + path + "'.");
  } catch (const YAML::ParserException & ex) {
    throw std::invalid_argument("Failed to parse '" + path + "': " + ex.what());
  }
  if (!root.IsMap()) {
    throw std::invalid_argument("Config file '" + path + "' must be a YAML mapping.");
  }

  if (const YAML::Node robot = root["robot"]) {
    if (const YAML::Node v = robot["hostname"]) config.robot_hostname = v.as<std::string>();
  }
  if (const YAML::Node wrench = root["wrench"]) {
    if (const YAML::Node v = wrench["source"]) config.wrench_source = v.as<std::string>();
    if (const YAML::Node v = wrench["serial_port"]) config.serial_port = v.as<std::string>();
    if (const YAML::Node v = wrench["topic"]) config.wrench_topic = v.as<std::string>();
    if (const YAML::Node v = wrench["frame"]) config.wrench_frame = v.as<std::string>();
    if (const YAML::Node v = wrench["sign"]) config.wrench_sign = asDouble(v, "wrench.sign");
    if (const YAML::Node v = wrench["filter_alpha"]) {
      config.wrench_filter_alpha = asDouble(v, "wrench.filter_alpha");
    }
  }
  if (const YAML::Node geometry = root["tool_geometry"]) {
    if (const YAML::Node v = geometry["sensor_mount_rpy_deg"]) {
      config.sensor_mount_rpy_deg = asVector3(v, "tool_geometry.sensor_mount_rpy_deg");
    }
    if (const YAML::Node v = geometry["flange_to_tcp"]) {
      config.flange_to_tcp = asVector3(v, "tool_geometry.flange_to_tcp");
    }
    if (const YAML::Node v = geometry["sensor_to_tcp"]) {
      config.sensor_to_tcp = asVector3(v, "tool_geometry.sensor_to_tcp");
    }
  }
  if (const YAML::Node payload = root["payload"]) {
    if (const YAML::Node v = payload["enabled"]) {
      config.set_payload = asBool(v, "payload.enabled");
    }
    if (const YAML::Node v = payload["mass"]) {
      config.payload_mass = asDouble(v, "payload.mass");
    }
    if (const YAML::Node v = payload["center_of_mass"]) {
      config.payload_center_of_mass = asVector3(v, "payload.center_of_mass");
    }
    if (const YAML::Node v = payload["inertia"]) {
      config.payload_inertia = asVector9(v, "payload.inertia");
    }
  }
  if (const YAML::Node adm = root["admittance"]) {
    if (const YAML::Node v = adm["frame"]) {
      config.admittance_frame = v.as<std::string>();
    }
    applyAdmittance(adm, config, "admittance");
  }
  if (const YAML::Node ji = root["joint_impedance"]) {
    if (const YAML::Node v = ji["stiffness"]) {
      config.joint_stiffness = asVector7(v, "joint_impedance.stiffness");
    }
    if (const YAML::Node v = ji["damping"]) {
      config.joint_damping = asVector7(v, "joint_impedance.damping");
    }
  }
  if (const YAML::Node lim = root["limits"]) {
    if (const YAML::Node v = lim["force_deadband"]) {
      config.force_deadband = asDouble(v, "limits.force_deadband");
    }
    if (const YAML::Node v = lim["max_force"]) {
      config.max_external_force = asDouble(v, "limits.max_force");
    }
    if (const YAML::Node v = lim["torque_deadband"]) {
      config.torque_deadband = asDouble(v, "limits.torque_deadband");
    }
    if (const YAML::Node v = lim["max_torque"]) {
      config.max_external_torque = asDouble(v, "limits.max_torque");
    }
    if (const YAML::Node v = lim["max_translation_offset"]) {
      config.max_translation_offset = asDouble(v, "limits.max_translation_offset");
    }
    if (const YAML::Node v = lim["max_rotation_offset"]) {
      config.max_rotation_offset = asDouble(v, "limits.max_rotation_offset");
    }
    if (const YAML::Node v = lim["max_linear_speed"]) {
      config.max_linear_speed = asDouble(v, "limits.max_linear_speed");
    }
    if (const YAML::Node v = lim["max_angular_speed"]) {
      config.max_angular_speed = asDouble(v, "limits.max_angular_speed");
    }
  }
  if (const YAML::Node log = root["logging"]) {
    if (const YAML::Node v = log["log_pose_error"]) {
      config.log_pose_error = asBool(v, "logging.log_pose_error");
    }
    if (const YAML::Node v = log["pose_log_dir"]) config.pose_log_dir = v.as<std::string>();
    if (const YAML::Node v = log["auto_plot"]) config.auto_plot = asBool(v, "logging.auto_plot");
    if (const YAML::Node v = log["plot_output_dir"]) config.plot_output_dir = v.as<std::string>();
    if (const YAML::Node v = log["plot_script"]) config.plot_script = v.as<std::string>();
    if (const YAML::Node v = log["print_pose_error"]) {
      config.print_pose_error = asBool(v, "logging.print_pose_error");
    }
    if (const YAML::Node v = log["print_wrench_debug"]) {
      config.print_wrench_debug = asBool(v, "logging.print_wrench_debug");
    }
    if (const YAML::Node v = log["print_period"]) {
      config.print_period = asDouble(v, "logging.print_period");
    }
    if (const YAML::Node v = log["plot_filter_comparison"]) {
      config.plot_filter_comparison = asBool(v, "logging.plot_filter_comparison");
    }
  }

  std::optional<std::string> selected = requested_profile;
  if (!selected) {
    if (const YAML::Node dp = root["default_profile"]) selected = dp.as<std::string>();
  }
  if (!selected || selected->empty()) {
    return "";
  }
  const YAML::Node profiles = root["profiles"];
  if (!profiles || !profiles.IsMap()) {
    throw std::invalid_argument(
      "Profile '" + *selected + " requested but the config has no 'profiles:' section.");
  }
  const YAML::Node profile = profiles[*selected];
  if (!profile) {
    std::string available;
    for (const auto & kv : profiles) {
      if (!available.empty()) available += ", ";
      available += kv.first.as<std::string>();
    }
    throw std::invalid_argument(
      "Config has no profile '" + *selected + "'. Available profiles: " + available + ".");
  }
  applyAdmittance(profile, config, "profiles." + *selected);
  return *selected;
}

bool parseBoolArg(const std::string & raw, const std::string & flag) {
  std::string lower;
  lower.reserve(raw.size());
  for (char c : raw) {
    lower += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  if (lower == "true" || lower == "1" || lower == "yes" || lower == "on") return true;
  if (lower == "false" || lower == "0" || lower == "no" || lower == "off") return false;
  throw std::invalid_argument(flag + " expects a boolean (true/false), got '" + raw + "'.");
}

double parseDoubleArg(const std::string & raw, const std::string & flag) {
  try {
    size_t parsed = 0;
    const double value = std::stod(raw, &parsed);
    if (parsed != raw.size() || !std::isfinite(value)) {
      throw std::invalid_argument("bad value");
    }
    return value;
  } catch (const std::exception &) {
    throw std::invalid_argument(flag + " expects a finite number, got '" + raw + "'.");
  }
}

int parseIntArg(const std::string & raw, const std::string & flag) {
  try {
    size_t parsed = 0;
    const int value = std::stoi(raw, &parsed);
    if (parsed != raw.size()) {
      throw std::invalid_argument("bad value");
    }
    return value;
  } catch (const std::exception &) {
    throw std::invalid_argument(flag + " expects an integer, got '" + raw + "'.");
  }
}

std::array<double, 7> parseVector7Arg(const std::string & raw, const std::string & flag) {
  std::array<double, 7> values{};
  std::stringstream ss(raw);
  std::string field;
  size_t i = 0;
  while (std::getline(ss, field, ',')) {
    if (i >= values.size()) {
      throw std::invalid_argument(flag + " expects exactly 7 comma-separated numbers.");
    }
    values[i++] = parseDoubleArg(field, flag);
  }
  if (i != values.size()) {
    throw std::invalid_argument(flag + " expects exactly 7 comma-separated numbers.");
  }
  return values;
}

void validateConfig(const CartesianAdmittanceConfig & config) {
  if (config.wrench_source != "franka" && config.wrench_source != "topic" &&
      config.wrench_source != "serial") {
    throw std::invalid_argument("wrench.source must be 'franka', 'topic', or 'serial'.");
  }
  if (config.wrench_frame != "local" && config.wrench_frame != "world") {
    throw std::invalid_argument("wrench.frame must be either 'local' or 'world'.");
  }
  if (!std::isfinite(config.wrench_sign)) {
    throw std::invalid_argument("wrench.sign must be finite.");
  }
  if (config.admittance_frame != "actual_tcp") {
    throw std::invalid_argument(
      "admittance.frame must be 'actual_tcp' (moving measured-TCP frame).");
  }
  const auto validateVector3 = [](const std::array<double, 3> & values, const char * key) {
    for (size_t i = 0; i < values.size(); ++i) {
      if (!std::isfinite(values[i])) {
        throw std::invalid_argument(
          std::string(key) + "[" + std::to_string(i) + "] must be finite.");
      }
    }
  };
  validateVector3(config.sensor_mount_rpy_deg, "tool_geometry.sensor_mount_rpy_deg");
  validateVector3(config.flange_to_tcp, "tool_geometry.flange_to_tcp");
  validateVector3(config.sensor_to_tcp, "tool_geometry.sensor_to_tcp");
  if (!std::isfinite(config.payload_mass) || config.payload_mass < 0.0) {
    throw std::invalid_argument("payload.mass must be finite and non-negative.");
  }
  validateVector3(config.payload_center_of_mass, "payload.center_of_mass");
  for (size_t i = 0; i < config.payload_inertia.size(); ++i) {
    if (!std::isfinite(config.payload_inertia[i])) {
      throw std::invalid_argument(
        "payload.inertia[" + std::to_string(i) + "] must be finite.");
    }
  }
  if (config.set_payload) {
    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::ColMajor>> inertia(
      config.payload_inertia.data());
    if (!inertia.isApprox(inertia.transpose(), 1.0e-12)) {
      throw std::invalid_argument("payload.inertia must be symmetric.");
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(inertia);
    if (eigensolver.info() != Eigen::Success || eigensolver.eigenvalues().minCoeff() <= 0.0) {
      throw std::invalid_argument(
        "payload.inertia must be positive definite; Franka Control rejects a zero matrix.");
    }
    const Eigen::Vector3d principal = eigensolver.eigenvalues();
    if (principal(2) > principal(0) + principal(1) + 1.0e-12) {
      throw std::invalid_argument(
        "payload.inertia principal moments must satisfy Imax <= Iother1 + Iother2.");
    }
  }
  const auto validateJointGains =
    [](const std::array<double, 7> & gains, const char * key) {
      for (int i = 0; i < 7; ++i) {
        const double value = gains[static_cast<size_t>(i)];
        if (!std::isfinite(value) || value < 0.0) {
          throw std::invalid_argument(
            std::string(key) + "[" + std::to_string(i) + "] for joint " +
            std::to_string(i + 1) + " must be finite and non-negative.");
        }
      }
    };
  validateJointGains(config.joint_stiffness, "joint_impedance.stiffness");
  validateJointGains(config.joint_damping, "joint_impedance.damping");
}

void validateTrajectory(const TrajectoryParams & trajectory) {
  if (trajectory.step <= 0.0 || trajectory.step > 0.10) {
    throw std::invalid_argument("--step must be in (0, 0.10] meters.");
  }
  if (trajectory.circle_radius <= 0.0 || trajectory.circle_radius > 0.10) {
    throw std::invalid_argument("--circle-radius must be in (0, 0.10] meters.");
  }
  if (trajectory.segment_time <= 0.0) {
    throw std::invalid_argument("--segment-time must be positive.");
  }
  if (trajectory.hold_time < 0.0) {
    throw std::invalid_argument("--hold-time must be non-negative.");
  }
  if (trajectory.circle_time <= 0.0) {
    throw std::invalid_argument("--circle-time must be positive.");
  }
  if (trajectory.circle_revolutions < 1) {
    throw std::invalid_argument("--circle-revolutions must be at least 1.");
  }
}

double minimumJerk01(double u) {
  const double s = std::clamp(u, 0.0, 1.0);
  return s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
}

std::string trim(const std::string & value) {
  const size_t first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const size_t last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::vector<double> parseCsvNumbers(const std::string & line) {
  std::vector<double> values;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    field = trim(field);
    if (field.empty()) {
      throw std::invalid_argument("empty CSV field");
    }
    size_t parsed = 0;
    const double value = std::stod(field, &parsed);
    if (parsed != field.size() || !std::isfinite(value)) {
      throw std::invalid_argument("non-finite CSV field '" + field + "'");
    }
    values.push_back(value);
  }
  return values;
}

CsvTrajectory loadCsvTrajectory(
  const std::string & path, double sample_period, bool relative) {
  if (sample_period <= 0.0) {
    throw std::invalid_argument("--sample-period must be positive.");
  }

  std::ifstream in(path);
  if (!in) {
    throw std::invalid_argument("Could not open trajectory CSV '" + path + "'.");
  }

  CsvTrajectory trajectory;
  trajectory.path = path;
  trajectory.relative = relative;

  std::string line;
  size_t line_number = 0;
  size_t sample_index = 0;
  while (std::getline(in, line)) {
    ++line_number;
    line = trim(line);
    if (line.empty() || line[0] == '#') continue;

    std::vector<double> values;
    try {
      values = parseCsvNumbers(line);
    } catch (const std::exception &) {
      if (trajectory.samples.empty()) {
        continue;  // likely header line
      }
      throw std::invalid_argument(
        "Bad numeric value in '" + path + "' line " + std::to_string(line_number) + ".");
    }

    if (values.size() != 7 && values.size() != 8) {
      throw std::invalid_argument(
        "Trajectory CSV '" + path + "' line " + std::to_string(line_number) +
        " must contain either x,y,z,qx,qy,qz,qw or t,x,y,z,qx,qy,qz,qw.");
    }

    const size_t offset = values.size() == 8 ? 1 : 0;
    PoseTrajectorySample sample;
    sample.t = values.size() == 8 ? values[0] : sample_period * static_cast<double>(sample_index);
    sample.position = Eigen::Vector3d(values[offset], values[offset + 1], values[offset + 2]);
    sample.orientation =
      Eigen::Quaterniond(values[offset + 6], values[offset + 3], values[offset + 4], values[offset + 5]);
    const double q_norm = sample.orientation.norm();
    if (q_norm < 1.0e-9) {
      throw std::invalid_argument(
        "Trajectory CSV '" + path + "' line " + std::to_string(line_number) +
        " has a zero quaternion.");
    }
    sample.orientation.normalize();

    if (!trajectory.samples.empty()) {
      if (sample.t <= trajectory.samples.back().t) {
        throw std::invalid_argument(
          "Trajectory CSV '" + path + "' line " + std::to_string(line_number) +
          " has non-increasing time.");
      }
      if (trajectory.samples.back().orientation.dot(sample.orientation) < 0.0) {
        sample.orientation.coeffs() *= -1.0;
      }
    }
    trajectory.samples.push_back(sample);
    ++sample_index;
  }

  if (trajectory.samples.size() < 2) {
    throw std::invalid_argument("Trajectory CSV '" + path + "' must contain at least 2 samples.");
  }
  return trajectory;
}

double csvTrajectoryDuration(
  const CsvTrajectory & trajectory,
  double hold_time,
  double prealign_time,
  double final_return_time,
  double post_hold_time) {
  return hold_time + prealign_time + trajectory.samples.back().t + final_return_time +
         post_hold_time;
}

PoseTrajectorySample interpolateCsvTrajectory(const CsvTrajectory & trajectory, double t) {
  if (t <= trajectory.samples.front().t) return trajectory.samples.front();
  if (t >= trajectory.samples.back().t) return trajectory.samples.back();

  const auto upper = std::upper_bound(
    trajectory.samples.begin(),
    trajectory.samples.end(),
    t,
    [](double value, const PoseTrajectorySample & sample) { return value < sample.t; });
  const auto lower = upper - 1;
  const double u = (t - lower->t) / (upper->t - lower->t);

  PoseTrajectorySample out;
  out.t = t;
  out.position = lower->position + u * (upper->position - lower->position);
  out.orientation = lower->orientation.slerp(u, upper->orientation);
  out.orientation.normalize();
  return out;
}

void sampleCsvTrajectory(
  const CsvTrajectory & trajectory,
  double hold_time,
  double prealign_time,
  double final_return_time,
  double elapsed_time,
  const Eigen::Vector3d & start,
  const Eigen::Matrix3d & start_rotation,
  Eigen::Vector3d & target_position,
  Eigen::Matrix3d & target_rotation) {
  target_position = start;
  target_rotation = start_rotation;
  if (elapsed_time <= hold_time) {
    return;
  }

  double t = elapsed_time - hold_time;
  if (prealign_time > 0.0 && t <= prealign_time) {
    const double u = minimumJerk01(t / prealign_time);
    const PoseTrajectorySample & first = trajectory.samples.front();
    target_position = start;
    if (!trajectory.relative) {
      target_position = start + u * (first.position - start);
    }
    Eigen::Quaterniond q_start(start_rotation);
    Eigen::Quaterniond q_goal = first.orientation;
    if (q_start.dot(q_goal) < 0.0) {
      q_goal.coeffs() *= -1.0;
    }
    Eigen::Quaterniond q_target = q_start.slerp(u, q_goal);
    q_target.normalize();
    target_rotation = q_target.toRotationMatrix();
    return;
  }
  t -= prealign_time;

  const double playback_duration = trajectory.samples.back().t;
  const PoseTrajectorySample sample = interpolateCsvTrajectory(trajectory, t);
  auto assign_playback_target = [&](const PoseTrajectorySample & playback_sample) {
    if (!trajectory.relative) {
      target_position = playback_sample.position;
      target_rotation = playback_sample.orientation.toRotationMatrix();
      return;
    }

    const PoseTrajectorySample & first = trajectory.samples.front();
    target_position = start + (playback_sample.position - first.position);
    target_rotation = playback_sample.orientation.toRotationMatrix();
  };

  if (t <= playback_duration) {
    assign_playback_target(sample);
    return;
  }

  PoseTrajectorySample last = trajectory.samples.back();
  if (final_return_time <= 0.0) {
    assign_playback_target(last);
    return;
  }

  Eigen::Vector3d return_start_position = last.position;
  Eigen::Quaterniond return_start_orientation = last.orientation;
  if (trajectory.relative) {
    const PoseTrajectorySample & first = trajectory.samples.front();
    return_start_position = start + (last.position - first.position);
  }

  const double return_elapsed = t - playback_duration;
  if (return_elapsed <= final_return_time) {
    const double u = minimumJerk01(return_elapsed / final_return_time);
    target_position = return_start_position + u * (start - return_start_position);
    Eigen::Quaterniond q_start = return_start_orientation;
    Eigen::Quaterniond q_goal(start_rotation);
    if (q_start.dot(q_goal) < 0.0) {
      q_goal.coeffs() *= -1.0;
    }
    Eigen::Quaterniond q_target = q_start.slerp(u, q_goal);
    q_target.normalize();
    target_rotation = q_target.toRotationMatrix();
    return;
  }

  target_position = start;
  target_rotation = start_rotation;
}

Eigen::Vector3d interpolateLine(
  const Eigen::Vector3d & from,
  const Eigen::Vector3d & to,
  double local_time,
  double duration) {
  return from + minimumJerk01(local_time / duration) * (to - from);
}

double trajectoryDuration(const TrajectoryParams & params) {
  return 2.0 * params.hold_time + 10.0 * params.segment_time + params.circle_time;
}

void sampleTrajectory(
  const TrajectoryParams & params,
  double elapsed_time,
  const Eigen::Vector3d & start,
  const Eigen::Matrix3d & start_rotation,
  Eigen::Vector3d & target_position,
  Eigen::Matrix3d & target_rotation) {
  const Eigen::Vector3d forward = start + Eigen::Vector3d(params.step, 0.0, 0.0);
  const Eigen::Vector3d backward = start + Eigen::Vector3d(-params.step, 0.0, 0.0);
  const Eigen::Vector3d right = start + Eigen::Vector3d(0.0, -params.step, 0.0);
  const Eigen::Vector3d left = start + Eigen::Vector3d(0.0, params.step, 0.0);
  const Eigen::Vector3d circle_start =
    start + Eigen::Vector3d(params.circle_radius, 0.0, 0.0);

  target_rotation = start_rotation;
  target_position = start;

  double t = elapsed_time;
  if (t <= params.hold_time) {
    return;
  }
  t -= params.hold_time;

  const auto line_phase =
    [&](const Eigen::Vector3d & from, const Eigen::Vector3d & to) -> bool {
      if (t <= params.segment_time) {
        target_position = interpolateLine(from, to, t, params.segment_time);
        return true;
      }
      t -= params.segment_time;
      return false;
    };

  if (line_phase(start, forward)) return;
  if (line_phase(forward, start)) return;
  if (line_phase(start, backward)) return;
  if (line_phase(backward, start)) return;
  if (line_phase(start, right)) return;
  if (line_phase(right, start)) return;
  if (line_phase(start, left)) return;
  if (line_phase(left, start)) return;
  if (line_phase(start, circle_start)) return;

  if (t <= params.circle_time) {
    const double s = minimumJerk01(t / params.circle_time);
    const double theta = 2.0 * kPi * static_cast<double>(params.circle_revolutions) * s;
    target_position =
      start + Eigen::Vector3d(
                params.circle_radius * std::cos(theta),
                params.circle_radius * std::sin(theta),
                0.0);
    return;
  }
  t -= params.circle_time;

  if (line_phase(circle_start, start)) return;
  target_position = start;
}

struct CliArgs {
  bool show_help = false;
  std::string config_path = kDefaultConfigPath;
  std::optional<std::string> robot_hostname;
  std::optional<std::string> profile;
  std::optional<bool> plot_filter_comparison;
  TrajectoryParams trajectory;
  std::optional<std::string> trajectory_csv;
  bool relative_csv = true;
  double sample_period = 0.05;
  double prealign_time = 3.0;
  double final_return_time = 5.0;
  double post_hold_time = 3.0;
  std::optional<double> max_linear_speed;
  std::optional<double> max_angular_speed;
  std::optional<std::array<double, 7>> initial_joints;
  double joint_prepose_time = 8.0;
  bool auto_start = false;
};

CliArgs parseCli(const std::vector<std::string> & args) {
  CliArgs cli;
  size_t i = 1;
  if (i < args.size() && !args[i].empty() && args[i].front() != '-') {
    cli.robot_hostname = args[i++];
  }
  for (; i < args.size(); ++i) {
    const std::string & opt = args[i];
    const auto require_value = [&](const std::string & flag) -> const std::string & {
      if (i + 1 >= args.size()) {
        throw std::invalid_argument(flag + " requires a value.");
      }
      return args[++i];
    };

    if (opt == "--help" || opt == "-h") {
      cli.show_help = true;
    } else if (opt == "--config") {
      cli.config_path = require_value(opt);
    } else if (opt == "--robot-hostname") {
      cli.robot_hostname = require_value(opt);
    } else if (opt == "--profile") {
      cli.profile = require_value(opt);
    } else if (opt == "--plot-filter-comparison") {
      cli.plot_filter_comparison = parseBoolArg(require_value(opt), opt);
    } else if (opt == "--trajectory-csv") {
      cli.trajectory_csv = require_value(opt);
    } else if (opt == "--relative") {
      cli.relative_csv = true;
    } else if (opt == "--absolute") {
      cli.relative_csv = false;
    } else if (opt == "--sample-period") {
      cli.sample_period = parseDoubleArg(require_value(opt), opt);
    } else if (opt == "--prealign-time") {
      cli.prealign_time = parseDoubleArg(require_value(opt), opt);
    } else if (opt == "--final-return-time") {
      cli.final_return_time = parseDoubleArg(require_value(opt), opt);
    } else if (opt == "--post-hold-time") {
      cli.post_hold_time = parseDoubleArg(require_value(opt), opt);
    } else if (opt == "--max-linear-speed") {
      cli.max_linear_speed = parseDoubleArg(require_value(opt), opt);
    } else if (opt == "--max-angular-speed") {
      cli.max_angular_speed = parseDoubleArg(require_value(opt), opt);
    } else if (opt == "--initial-joints") {
      cli.initial_joints = parseVector7Arg(require_value(opt), opt);
    } else if (opt == "--joint-prepose-time") {
      cli.joint_prepose_time = parseDoubleArg(require_value(opt), opt);
    } else if (opt == "--auto-start") {
      cli.auto_start = true;
    } else if (opt == "--step") {
      cli.trajectory.step = parseDoubleArg(require_value(opt), opt);
    } else if (opt == "--circle-radius") {
      cli.trajectory.circle_radius = parseDoubleArg(require_value(opt), opt);
    } else if (opt == "--segment-time") {
      cli.trajectory.segment_time = parseDoubleArg(require_value(opt), opt);
    } else if (opt == "--hold-time") {
      cli.trajectory.hold_time = parseDoubleArg(require_value(opt), opt);
    } else if (opt == "--circle-time") {
      cli.trajectory.circle_time = parseDoubleArg(require_value(opt), opt);
    } else if (opt == "--circle-revolutions") {
      cli.trajectory.circle_revolutions = parseIntArg(require_value(opt), opt);
    } else {
      throw std::invalid_argument("Unknown option '" + opt + "'. Use --help for usage.");
    }
  }
  validateTrajectory(cli.trajectory);
  if (cli.sample_period <= 0.0) {
    throw std::invalid_argument("--sample-period must be positive.");
  }
  if (cli.prealign_time < 0.0) {
    throw std::invalid_argument("--prealign-time must be non-negative.");
  }
  if (cli.final_return_time < 0.0) {
    throw std::invalid_argument("--final-return-time must be non-negative.");
  }
  if (cli.post_hold_time < 0.0) {
    throw std::invalid_argument("--post-hold-time must be non-negative.");
  }
  if (cli.max_linear_speed && *cli.max_linear_speed <= 0.0) {
    throw std::invalid_argument("--max-linear-speed must be positive.");
  }
  if (cli.max_angular_speed && *cli.max_angular_speed <= 0.0) {
    throw std::invalid_argument("--max-angular-speed must be positive.");
  }
  if (cli.joint_prepose_time <= 0.0) {
    throw std::invalid_argument("--joint-prepose-time must be positive.");
  }
  return cli;
}

bool moveToInitialJoints(
  const std::string & robot_hostname,
  const std::array<double, 7> & target_joints,
  double duration) {
  try {
    franka::Robot robot(robot_hostname);
    robot.automaticErrorRecovery();

    std::array<double, 7> q_start{};
    bool initialized = false;
    double elapsed = 0.0;

    auto callback =
      [&](const franka::RobotState & state, franka::Duration period) -> franka::JointPositions {
        if (!initialized) {
          q_start = state.q;
          initialized = true;
        }
        elapsed += period.toSec();
        const double s = minimumJerk01(elapsed / duration);
        std::array<double, 7> q_cmd{};
        for (size_t i = 0; i < q_cmd.size(); ++i) {
          q_cmd[i] = q_start[i] + s * (target_joints[i] - q_start[i]);
        }
        franka::JointPositions command(q_cmd);
        if (elapsed >= duration) {
          return franka::MotionFinished(command);
        }
        return command;
      };

    std::cout << "Moving to recorded initial joints over " << duration << " s..." << std::endl;
    robot.control(callback);
    std::cout << "Reached recorded initial joint prepose." << std::endl;
    return true;
  } catch (const franka::Exception & ex) {
    std::cerr << "libfranka joint prepose error: " << ex.what() << std::endl;
    return false;
  } catch (const std::exception & ex) {
    std::cerr << "Joint prepose error: " << ex.what() << std::endl;
    return false;
  }
}

void printHelp(const std::string & program) {
  std::cout <<
    "Usage: " << program
              << " [robot-hostname] [--config <path>] [--profile <name>]\n"
    "                             [--trajectory-csv <path>] [--relative|--absolute]\n"
    "                             [--sample-period <s>] [--prealign-time <s>]\n"
    "                             [--final-return-time <s>] [--post-hold-time <s>]\n"
    "                             [--max-linear-speed <m/s>] [--max-angular-speed <rad/s>]\n"
    "                             [--initial-joints <q1,...,q7>] [--joint-prepose-time <s>]\n"
    "                             [--auto-start]\n"
    "                             [--step <m>] [--circle-radius <m>]\n"
    "                             [--segment-time <s>] [--hold-time <s>]\n"
    "                             [--circle-time <s>] [--circle-revolutions <n>]\n"
    "                             [--plot-filter-comparison <bool>] [--help]\n"
    "\n"
    "Runs the tuned Cartesian admittance controller from the YAML config. With\n"
    "--trajectory-csv, the CSV must contain either x,y,z,qx,qy,qz,qw or\n"
    "t,x,y,z,qx,qy,qz,qw. CSV playback is relative by default for XYZ only: the first\n"
    "recorded position is mapped onto the measured TCP position, while recorded\n"
    "quaternions are replayed faithfully. Before playback, the robot smoothly aligns to\n"
    "the first recorded quaternion over --prealign-time seconds; after playback, it\n"
    "returns to the measured start pose over --final-return-time seconds. If\n"
    "--final-return-time is 0, it holds the final trajectory pose instead. Without\n"
    "--trajectory-csv, it runs the built-in +X/-X/-Y/+Y plus small-circle test.\n"
    "\n"
    "Example:\n"
    "  " << program << " 10.90.90.10 --profile soft\n"
    "  " << program << " 10.90.90.10 --profile soft --trajectory-csv /tmp/traj.csv\n"
    "  " << program << " 10.90.90.10 --step 0.02 --circle-radius 0.015 --segment-time 3.0\n"
    << std::endl;
}

void printSummary(
  const CartesianAdmittanceConfig & config,
  const std::string & config_path,
  const std::string & profile,
  const TrajectoryParams & trajectory,
  const std::optional<CsvTrajectory> & csv_trajectory,
  double prealign_time,
  double final_return_time,
  double post_hold_time,
  bool auto_start) {
  std::cout << "WARNING: This program directly controls the robot through libfranka.\n"
            << "Stop any ROS2 franka hardware/controller_manager launch before running it.\n"
            << "Franka error recovery will be requested before starting control.\n"
            << "Config: " << config_path << "\n"
            << "Profile: " << (profile.empty() ? "none" : profile) << "\n"
            << "Robot: " << config.robot_hostname << "\n"
            << "Wrench source: " << config.wrench_source << " (frame " << config.wrench_frame
            << ", sign " << config.wrench_sign << ")\n"
            << "Admittance frame: measured TCP " << config.admittance_frame
            << " (external-wrench offset only)\n";
  if (config.wrench_source == "serial") {
    std::cout << "Serial port: " << config.serial_port << "\n";
  }
  if (config.wrench_source == "topic") {
    std::cout << "Wrench topic: " << config.wrench_topic << "\n";
  }
  if (csv_trajectory) {
    const auto & samples = csv_trajectory->samples;
    std::cout << "CSV trajectory: " << csv_trajectory->path << "\n"
              << "Samples: " << samples.size() << ", playback duration "
              << samples.back().t << " s, mode "
              << (csv_trajectory->relative ? "relative" : "absolute") << "\n"
              << "Pre-align to first recorded orientation: " << prealign_time << " s\n"
              << "Final return to measured start pose: " << final_return_time << " s\n"
              << "Post-trajectory hold before finish: " << post_hold_time << " s\n"
              << "Start pose [x y z]: [" << samples.front().position.x() << ", "
              << samples.front().position.y() << ", " << samples.front().position.z()
              << "]\n"
              << "End pose [x y z]: [" << samples.back().position.x() << ", "
              << samples.back().position.y() << ", " << samples.back().position.z()
              << "]\n";
  } else {
    std::cout << "Trajectory: hold " << trajectory.hold_time << " s, step "
              << trajectory.step << " m, segment " << trajectory.segment_time
              << " s, circle radius " << trajectory.circle_radius << " m, circle "
              << trajectory.circle_time << " s x " << trajectory.circle_revolutions
              << " revolution(s)\n"
              << "World-frame directions: forward +X, backward -X, right -Y, left +Y\n";
  }
  std::cout << "Total run time: " << config.run_time << " s\n";
  if (auto_start) {
    std::cout << "Start mode: auto-start requested by caller." << std::endl;
  } else {
    std::cout << "Press Enter to start..." << std::endl;
  }
}

}  // namespace

int main(int argc, char ** argv) {
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  const std::string program = args.empty() ? "cartesian_admittance_trajectory" : args[0];

  CliArgs cli;
  try {
    cli = parseCli(args);
  } catch (const std::exception & ex) {
    std::cerr << ex.what() << "\nUse --help for usage." << std::endl;
    return 1;
  }
  if (cli.show_help) {
    printHelp(program);
    return 0;
  }

  CartesianAdmittanceConfig config;
  std::string applied_profile;
  std::optional<CsvTrajectory> csv_trajectory;
  try {
    applied_profile = loadConfigFile(cli.config_path, cli.profile, config);
    if (cli.robot_hostname) {
      config.robot_hostname = *cli.robot_hostname;
    }
    if (cli.plot_filter_comparison) {
      config.plot_filter_comparison = *cli.plot_filter_comparison;
    }
    if (cli.max_linear_speed) {
      config.max_linear_speed = *cli.max_linear_speed;
    }
    if (cli.max_angular_speed) {
      config.max_angular_speed = *cli.max_angular_speed;
    }
    if (cli.trajectory_csv) {
      csv_trajectory = loadCsvTrajectory(*cli.trajectory_csv, cli.sample_period, cli.relative_csv);
      config.run_time =
        csvTrajectoryDuration(
          *csv_trajectory,
          cli.trajectory.hold_time,
          cli.prealign_time,
          cli.final_return_time,
          cli.post_hold_time);
    } else {
      config.run_time = trajectoryDuration(cli.trajectory);
    }
    config.stop_on_settled = false;
    const TrajectoryParams trajectory = cli.trajectory;
    const double prealign_time = cli.prealign_time;
    const double final_return_time = cli.final_return_time;
    if (csv_trajectory) {
      const CsvTrajectory trajectory_csv = *csv_trajectory;
      config.target_provider =
        [trajectory_csv, trajectory, prealign_time, final_return_time](
          double elapsed_time,
          const Eigen::Vector3d & initial_position_world,
          const Eigen::Matrix3d & initial_rotation_world,
          Eigen::Vector3d & target_position_world,
          Eigen::Matrix3d & target_rotation_world) {
          sampleCsvTrajectory(
            trajectory_csv,
            trajectory.hold_time,
            prealign_time,
            final_return_time,
            elapsed_time,
            initial_position_world,
            initial_rotation_world,
            target_position_world,
            target_rotation_world);
        };
    } else {
      config.target_provider =
        [trajectory](
          double elapsed_time,
          const Eigen::Vector3d & initial_position_world,
          const Eigen::Matrix3d & initial_rotation_world,
          Eigen::Vector3d & target_position_world,
          Eigen::Matrix3d & target_rotation_world) {
          sampleTrajectory(
            trajectory,
            elapsed_time,
            initial_position_world,
            initial_rotation_world,
            target_position_world,
            target_rotation_world);
        };
    }
    validateConfig(config);
  } catch (const std::exception & ex) {
    std::cerr << "Configuration error: " << ex.what() << std::endl;
    return 1;
  }

  printSummary(
    config, cli.config_path, applied_profile, cli.trajectory, csv_trajectory,
    cli.prealign_time, cli.final_return_time, cli.post_hold_time, cli.auto_start);
  if (cli.auto_start) {
    std::cout << "Auto-start enabled by caller; starting control now." << std::endl;
  } else {
    if (!isatty(STDIN_FILENO)) {
      std::cerr << "No interactive terminal was detected; not starting control." << std::endl;
      return 1;
    }
    std::cin.ignore();
    if (!std::cin) {
      std::cerr << "No Enter key was received; not starting control." << std::endl;
      return 1;
    }
  }

  if (cli.initial_joints) {
    if (!moveToInitialJoints(config.robot_hostname, *cli.initial_joints, cli.joint_prepose_time)) {
      return 1;
    }
  }

  return my_controller::runCartesianAdmittanceToTarget(config) ? 0 : 1;
}
