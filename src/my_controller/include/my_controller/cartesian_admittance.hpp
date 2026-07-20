// Reusable libfranka Cartesian admittance controller that ramps the TCP to a target
// pose (world frame) and then holds it while staying compliant. Include this header and
// link against the cartesian_admittance_lib target to drive the robot from other code.
#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <optional>
#include <string>

#include <Eigen/Dense>

namespace my_controller {

using CartesianTargetProvider = std::function<void(
  double elapsed_time,
  const Eigen::Vector3d & initial_position_world,
  const Eigen::Matrix3d & initial_rotation_world,
  Eigen::Vector3d & target_position_world,
  Eigen::Matrix3d & target_rotation_world)>;

// Optional real-time pose observer. It is called once per control cycle with the measured
// physical TCP pose and final compliant TCP command, both in world/base frame O. The observer
// must be non-blocking and allocation-free; it should only copy into preallocated/atomic storage.
using CartesianTcpPoseObserver = std::function<void(
  const Eigen::Vector3d & actual_position_world,
  const Eigen::Matrix3d & actual_rotation_world,
  const Eigen::Vector3d & command_position_world,
  const Eigen::Matrix3d & command_rotation_world)>;

using CartesianJointPositionObserver =
  std::function<void(const std::array<double, 7> & actual_joint_positions)>;

struct CartesianAdmittanceConfig {
  std::string robot_hostname = "10.90.90.10";

  std::string wrench_source = "serial";     // "franka", "topic", or "serial"
  std::string serial_port = "/dev/ttyACM0";
  std::string wrench_topic = "resense_0";
  std::string wrench_frame = "local";       // "local" or "world"
  double wrench_sign = 1.0;

  // F/T sensor mounting and rigid tool geometry. RPY is the sensor->flange coordinate
  // mapping in degrees, with R = Rz(yaw)*Ry(pitch)*Rx(roll). Translation vectors are in
  // meters and expressed in their named source frame.
  std::array<double, 3> sensor_mount_rpy_deg = {0.0, 0.0, -135.0};
  std::array<double, 3> flange_to_tcp = {0.0, 0.0, 0.1215};
  std::array<double, 3> sensor_to_tcp = {0.0, 0.0, 0.1066};

  // Optional physical payload passed to libfranka for gravity/dynamics compensation.
  // The center of mass is expressed in the flange frame [m]; inertia is about that center
  // of mass [kg*m^2], stored column-major. When enabled, the inertia must be symmetric,
  // positive definite and satisfy the principal-moment triangle inequality.
  bool set_payload = false;
  double payload_mass = 0.0;
  std::array<double, 3> payload_center_of_mass = {0.0, 0.0, 0.0};
  std::array<double, 9> payload_inertia = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Desired TCP position expressed in the world/base frame.
  Eigen::Vector3d target_position_world = Eigen::Vector3d(0.45, 0.0, 0.35);

  // Hold the physical TCP pose measured at controller startup. When true, this
  // overrides target_position_world and target_rotation_world.
  bool hold_initial_tcp_pose = false;

  // Optional desired TCP orientation in world. If unset, the initial TCP orientation
  // measured at the start of the run is held.
  std::optional<Eigen::Matrix3d> target_rotation_world;

  // Optional dynamic target generator. When set, the controller calls this once per
  // control cycle after measuring the initial TCP pose, and uses the returned world-frame
  // TCP pose instead of the fixed target/ramp above.
  CartesianTargetProvider target_provider;

  // Optional measured/commanded physical TCP observer for monitoring and visualization.
  CartesianTcpPoseObserver tcp_pose_observer;

  // Optional measured joint-position observer for monitoring and URDF visualization.
  CartesianJointPositionObserver joint_position_observer;

  // The admittance equation is evaluated in the moving, actually measured TCP frame. The
  // nominal world-frame trajectory bypasses this virtual dynamics and the compliant offset
  // converges to zero when no external wrench is present.
  std::string admittance_frame = "actual_tcp";

  // Admittance gains, order [Fx, Fy, Fz, Mx, My, Mz] in the actual TCP frame. The mask
  // enables (1.0) or disables (0.0) each measured TCP axis so one DOF can be tuned at a time.
  std::array<double, 6> admittance_mass = {2.0, 2.0, 2.0, 0.50, 0.50, 0.50};
  std::array<double, 6> admittance_stiffness = {800.0, 800.0, 800.0, 5.0, 5.0, 5.0};
  std::array<double, 6> admittance_damping = {60.0, 60.0, 60.0, 3.5, 3.5, 3.5};
  std::array<double, 6> admittance_mask = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

  // Joint-space impedance gains for the inner joint-impedance torque law, ordered
  // [joint1, joint2, joint3, joint4, joint5, joint6, joint7]. These are independent of the
  // Cartesian admittance profiles: --profile soft/hard only changes the Cartesian gains
  // above, never these. Defaults match the previously hard-coded gains, so an absent
  // joint_impedance YAML section preserves the original behavior. Units: Nm/rad
  // (stiffness), Nms/rad (damping).
  std::array<double, 7> joint_stiffness = {120.0, 120.0, 120.0, 120.0, 80.0, 80.0, 40.0};
  std::array<double, 7> joint_damping = {22.0, 22.0, 22.0, 22.0, 18.0, 18.0, 13.0};

  double max_translation_offset = 0.15;   // m
  double max_rotation_offset = 0.75;      // rad
  double max_linear_speed = 0.20;         // m/s
  double max_angular_speed = 1.0;         // rad/s

  double force_deadband = 2.0;            // N
  double max_external_force = 10.0;       // N, saturate measured force magnitude
  double torque_deadband = 0.3;           // Nm
  double max_external_torque = 2.0;       // Nm, saturate measured torque magnitude
  double wrench_filter_alpha = 0.05;

  double target_ramp_time = 3.0;          // s, minimum-jerk ramp to the target pose
  double run_time = -1.0;                 // s, negative means run until externally stopped

  // Settling detector (only armed after the ramp finishes): the pose counts as settled
  // once the measured TCP stays within these position/rotation thresholds and the
  // admittance internal velocity stays below settle_velocity_threshold for
  // settle_window seconds. stop_on_settled ends control as soon as that happens.
  double settle_window = 2.0;                 // s
  double settle_position_threshold = 0.001;   // m
  double settle_rotation_threshold = 0.01;    // rad
  double settle_velocity_threshold = 0.005;   // m/s and rad/s (inner admittance velocity)
  bool stop_on_settled = false;

  bool log_pose_error = false;                // record a pose-error CSV, dumped after stop
  std::string pose_log_dir = "src/my_controller/admittance_fitting";  // relative to shell cwd
  bool auto_plot = false;                     // plot pose-error CSV after control stops
  std::string plot_output_dir = "src/my_controller/admittance_fitting";
  std::string plot_script = "src/my_controller/admittance_fitting/plot_ft_and_pose_error.py";

  // Comparison logging (separate from the pose-error CSV/plot above): when enabled the
  // controller records the external force immediately before the low-pass filter and the
  // filter output for every control cycle, then writes a CSV and a four-subplot PNG under
  // scripts/wrench_filter_comparison after control stops. This is comparison-only; the
  // admittance loop keeps using the filtered wrench regardless of this flag.
  bool plot_filter_comparison = false;

  // Terminal live printing, independent of CSV logging. These only affect what the
  // non-real-time printer thread prints; the force sensor, wrench pipeline, bias removal,
  // mask, deadband, saturation, filtering and admittance control always run regardless.
  bool print_pose_error = true;               // print the periodic [pose] error line
  bool print_wrench_debug = false;            // print the periodic [ft_sensor] wrench line
  double print_period = 0.2;                  // s, terminal print period

  // Optional external stop flag; when set true the controller finishes cleanly.
  std::atomic_bool* stop_requested = nullptr;
};

// Runs the Cartesian admittance controller. On start it captures the current TCP pose,
// then ramps the internal desired pose from that pose to config.target_position_world
// (and target_rotation_world, or the initial orientation if unset) with a minimum-jerk
// profile over config.target_ramp_time, and holds the target afterwards while the
// admittance loop stays active. Blocks until run_time elapses, *stop_requested is set,
// ROS is shut down (Ctrl+C), or a franka exception is thrown. Exceptions are caught and
// reported internally, and the wrench history is dumped on a franka::ControlException.
// Returns true only when libfranka control returned normally.
bool runCartesianAdmittanceToTarget(const CartesianAdmittanceConfig& config);

// Build a rotation matrix from roll/pitch/yaw (rad): R = Rz(yaw)*Ry(pitch)*Rx(roll).
inline Eigen::Matrix3d rpyToRotationMatrix(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

// Inverse of rpyToRotationMatrix; returns [roll, pitch, yaw] (rad).
inline Eigen::Vector3d rotationMatrixToRPY(const Eigen::Matrix3d& rotation) {
  Eigen::Vector3d rpy;
  const double sin_pitch = -rotation(2, 0);
  const double cos_pitch =
      std::sqrt(rotation(0, 0) * rotation(0, 0) + rotation(1, 0) * rotation(1, 0));
  if (cos_pitch > 1.0e-9) {
    rpy.x() = std::atan2(rotation(2, 1), rotation(2, 2));  // roll
    rpy.y() = std::atan2(sin_pitch, cos_pitch);            // pitch
    rpy.z() = std::atan2(rotation(1, 0), rotation(0, 0));  // yaw
  } else {
    // Gimbal lock: pin yaw at 0 and fold the remaining rotation into roll.
    rpy.x() = std::atan2(-rotation(1, 2), rotation(1, 1));
    rpy.y() = std::atan2(sin_pitch, cos_pitch);
    rpy.z() = 0.0;
  }
  return rpy;
}

// Predefined real-robot "home" TCP pose in the world/base frame, used by
// --target-pose-source home. This is a fixed Cartesian pose -- NO forward kinematics, NO
// MuJoCo, NO joint keyframe. It is the Franka ready pose expressed as a world-frame TCP
// pose: the ready flange sits at ~[0.307, 0, 0.487] pointing down, so with the 12 cm tool
// offset the TCP is ~[0.307, 0, 0.367] with the tool pointing down (roll = pi). Edit these
// two values to match your robot's home if it differs.
inline Eigen::Vector3d homeTcpPosition() {
  return Eigen::Vector3d(0.307, 0.0, 0.367);
}
inline Eigen::Matrix3d homeTcpRotation() {
  return rpyToRotationMatrix(3.14159265358979323846, 0.0, 0.0);  // tool pointing down
}

}  // namespace my_controller
