// Hold the physical TCP at the startup pose or a configured absolute target
// while Cartesian admittance remains active, and publish measured, commanded,
// and hold-target TCP frames for RViz verification.

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include "my_controller/cartesian_admittance.hpp"

#ifndef TCP_ADMITTANCE_DEFAULT_CONFIG
#define TCP_ADMITTANCE_DEFAULT_CONFIG "cartesian_admittance_tcp.yaml"
#endif

#ifndef TCP_ADMITTANCE_DEFAULT_LOG_DIR
#define TCP_ADMITTANCE_DEFAULT_LOG_DIR "tcp_admittance_verification_logs"
#endif

namespace {

using my_controller::CartesianAdmittanceConfig;

struct CliArgs {
  std::string config_path = TCP_ADMITTANCE_DEFAULT_CONFIG;
  std::optional<std::string> robot_hostname;
  std::optional<std::string> profile;
  std::string base_frame = "fr3_link0";
  std::string actual_tcp_frame = "tcp_actual";
  std::string command_tcp_frame = "tcp_command";
  std::string home_tcp_frame = "tcp_home";
  double tf_rate = 30.0;
  bool execute = false;
  bool show_help = false;
};

double parseFiniteDouble(const std::string &text, const std::string &option) {
  try {
    size_t parsed = 0;
    const double value = std::stod(text, &parsed);
    if (parsed != text.size() || !std::isfinite(value)) {
      throw std::invalid_argument("not finite");
    }
    return value;
  } catch (const std::exception &) {
    throw std::invalid_argument(option + " requires a finite number, got '" +
                                text + "'.");
  }
}

CliArgs parseCli(int argc, char **argv) {
  CliArgs cli;
  for (int i = 1; i < argc; ++i) {
    const std::string option(argv[i]);
    const auto requireValue = [&](const std::string &name) -> std::string {
      if (i + 1 >= argc) {
        throw std::invalid_argument(name + " requires a value.");
      }
      return argv[++i];
    };

    if (option == "--help" || option == "-h") {
      cli.show_help = true;
    } else if (option == "--config") {
      cli.config_path = requireValue(option);
    } else if (option == "--robot-hostname") {
      cli.robot_hostname = requireValue(option);
    } else if (option == "--profile") {
      cli.profile = requireValue(option);
    } else if (option == "--base-frame") {
      cli.base_frame = requireValue(option);
    } else if (option == "--actual-tcp-frame") {
      cli.actual_tcp_frame = requireValue(option);
    } else if (option == "--command-tcp-frame") {
      cli.command_tcp_frame = requireValue(option);
    } else if (option == "--home-tcp-frame") {
      cli.home_tcp_frame = requireValue(option);
    } else if (option == "--tf-rate") {
      cli.tf_rate = parseFiniteDouble(requireValue(option), option);
    } else if (option == "--execute") {
      cli.execute = true;
    } else {
      throw std::invalid_argument("Unknown option '" + option +
                                  "'. Use --help.");
    }
  }

  if (cli.base_frame.empty() || cli.actual_tcp_frame.empty() ||
      cli.command_tcp_frame.empty() || cli.home_tcp_frame.empty()) {
    throw std::invalid_argument("TF frame names must not be empty.");
  }
  if (cli.actual_tcp_frame == cli.command_tcp_frame ||
      cli.actual_tcp_frame == cli.home_tcp_frame ||
      cli.command_tcp_frame == cli.home_tcp_frame) {
    throw std::invalid_argument(
        "Actual, command, and home TCP frame names must be different.");
  }
  if (cli.tf_rate <= 0.0 || cli.tf_rate > 200.0) {
    throw std::invalid_argument("--tf-rate must be in (0, 200] Hz.");
  }
  return cli;
}

template <size_t N>
std::array<double, N> readArray(const YAML::Node &node, const std::string &key,
                                const std::array<double, N> &fallback) {
  if (!node || !node[key])
    return fallback;
  const YAML::Node value = node[key];
  if (!value.IsSequence() || value.size() != N) {
    throw std::invalid_argument(key + " must contain exactly " +
                                std::to_string(N) + " values.");
  }
  std::array<double, N> result{};
  for (size_t i = 0; i < N; ++i) {
    result[i] = value[i].as<double>();
    if (!std::isfinite(result[i])) {
      throw std::invalid_argument(key + " contains a non-finite value.");
    }
  }
  return result;
}

template <typename T>
void readScalar(const YAML::Node &node, const char *key, T &destination) {
  if (node && node[key])
    destination = node[key].as<T>();
}

std::string loadConfig(const std::string &path,
                       const std::optional<std::string> &requested_profile,
                       CartesianAdmittanceConfig &config) {
  const YAML::Node root = YAML::LoadFile(path);

  const YAML::Node robot = root["robot"];
  readScalar(robot, "hostname", config.robot_hostname);

  const YAML::Node wrench = root["wrench"];
  readScalar(wrench, "source", config.wrench_source);
  readScalar(wrench, "serial_port", config.serial_port);
  readScalar(wrench, "topic", config.wrench_topic);
  readScalar(wrench, "frame", config.wrench_frame);
  readScalar(wrench, "sign", config.wrench_sign);
  readScalar(wrench, "filter_alpha", config.wrench_filter_alpha);

  const YAML::Node geometry = root["tool_geometry"];
  config.sensor_mount_rpy_deg = readArray<3>(geometry, "sensor_mount_rpy_deg",
                                             config.sensor_mount_rpy_deg);
  config.flange_to_tcp =
      readArray<3>(geometry, "flange_to_tcp", config.flange_to_tcp);
  config.sensor_to_tcp =
      readArray<3>(geometry, "sensor_to_tcp", config.sensor_to_tcp);

  const YAML::Node payload = root["payload"];
  readScalar(payload, "enabled", config.set_payload);
  readScalar(payload, "mass", config.payload_mass);
  config.payload_center_of_mass =
      readArray<3>(payload, "center_of_mass", config.payload_center_of_mass);
  config.payload_inertia =
      readArray<9>(payload, "inertia", config.payload_inertia);

  const YAML::Node target = root["target"];
  readScalar(target, "ramp_time", config.target_ramp_time);
  const YAML::Node pose = target["pose"];
  const YAML::Node pose_source = target["pose_source"];
  if (pose && pose_source) {
    throw std::invalid_argument(
        "target must define either pose or pose_source, not both.");
  }
  if (pose) {
    const auto values = readArray<6>(target, "pose", std::array<double, 6>{});
    config.target_position_world =
        Eigen::Vector3d(values[0], values[1], values[2]);
    config.target_rotation_world =
        my_controller::rpyToRotationMatrix(values[3], values[4], values[5]);
    config.hold_initial_tcp_pose = false;
  } else {
    const std::string source =
        pose_source ? pose_source.as<std::string>() : "home";
    if (source == "initial") {
      config.hold_initial_tcp_pose = true;
      config.target_rotation_world.reset();
    } else if (source == "home") {
      config.hold_initial_tcp_pose = false;
      config.target_position_world = my_controller::homeTcpPosition();
      config.target_rotation_world = my_controller::homeTcpRotation();
    } else {
      throw std::invalid_argument(
          "target.pose_source must be 'initial' or 'home'.");
    }
  }

  const YAML::Node admittance = root["admittance"];
  readScalar(admittance, "frame", config.admittance_frame);
  config.admittance_mask =
      readArray<6>(admittance, "mask", config.admittance_mask);
  config.admittance_mass =
      readArray<6>(admittance, "mass", config.admittance_mass);

  std::string profile;
  if (requested_profile) {
    profile = *requested_profile;
  } else if (root["default_profile"]) {
    profile = root["default_profile"].as<std::string>();
  }
  if (!profile.empty()) {
    const YAML::Node selected = root["profiles"][profile];
    if (!selected) {
      throw std::invalid_argument("Profile '" + profile +
                                  "' does not exist in " + path + ".");
    }
    config.admittance_stiffness =
        readArray<6>(selected, "stiffness", config.admittance_stiffness);
    config.admittance_damping =
        readArray<6>(selected, "damping", config.admittance_damping);
  }

  const YAML::Node joint = root["joint_impedance"];
  config.joint_stiffness =
      readArray<7>(joint, "stiffness", config.joint_stiffness);
  config.joint_damping = readArray<7>(joint, "damping", config.joint_damping);

  const YAML::Node limits = root["limits"];
  readScalar(limits, "force_deadband", config.force_deadband);
  readScalar(limits, "max_force", config.max_external_force);
  readScalar(limits, "torque_deadband", config.torque_deadband);
  readScalar(limits, "max_torque", config.max_external_torque);
  readScalar(limits, "max_translation_offset", config.max_translation_offset);
  readScalar(limits, "max_rotation_offset", config.max_rotation_offset);
  readScalar(limits, "max_linear_speed", config.max_linear_speed);
  readScalar(limits, "max_angular_speed", config.max_angular_speed);

  const YAML::Node logging = root["logging"];
  readScalar(logging, "print_period", config.print_period);

  // This program always holds the selected target until Ctrl+C and always
  // records/prints the verification data.
  config.target_provider = {};
  config.run_time = -1.0;
  config.stop_on_settled = false;
  config.log_pose_error = true;
  config.pose_log_dir = TCP_ADMITTANCE_DEFAULT_LOG_DIR;
  config.auto_plot = false;
  config.plot_filter_comparison = false;
  config.print_pose_error = true;
  config.print_wrench_debug = true;
  config.print_period = std::max(config.print_period, 0.1);
  return profile;
}

void validateConfig(const CartesianAdmittanceConfig &config) {
  if (config.wrench_source != "franka" && config.wrench_source != "serial" &&
      config.wrench_source != "topic") {
    throw std::invalid_argument(
        "wrench.source must be franka, serial, or topic.");
  }
  if (config.wrench_frame != "local" && config.wrench_frame != "world") {
    throw std::invalid_argument("wrench.frame must be local or world.");
  }
  if (config.admittance_frame != "actual_tcp" &&
      config.admittance_frame != "nominal_tcp" &&
      config.admittance_frame != "world") {
    throw std::invalid_argument(
        "admittance.frame must be actual_tcp, nominal_tcp, or world.");
  }
  if (config.target_ramp_time <= 0.0) {
    throw std::invalid_argument("target.ramp_time must be positive.");
  }
  if (!config.hold_initial_tcp_pose &&
      (!config.target_rotation_world ||
       !config.target_position_world.allFinite() ||
       !config.target_rotation_world->allFinite())) {
    throw std::invalid_argument("The absolute target TCP pose is invalid.");
  }
  if (!std::isfinite(config.wrench_filter_alpha) ||
      config.wrench_filter_alpha <= 0.0 || config.wrench_filter_alpha > 1.0) {
    throw std::invalid_argument("wrench.filter_alpha must be in (0, 1].");
  }
  for (size_t i = 0; i < config.admittance_mass.size(); ++i) {
    if (config.admittance_mass[i] <= 0.0 ||
        config.admittance_stiffness[i] < 0.0 ||
        config.admittance_damping[i] < 0.0) {
      throw std::invalid_argument(
          "Admittance mass must be positive; stiffness/damping non-negative.");
    }
  }
}

struct TcpPose {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
};

// Single-writer seqlock: the real-time callback only performs relaxed atomic
// stores. The TF thread retries if it observes a write in progress, so position
// and rotation stay coherent.
class AtomicTcpPose {
public:
  void write(const Eigen::Vector3d &position,
             const Eigen::Matrix3d &rotation) noexcept {
    sequence_.fetch_add(1, std::memory_order_acq_rel);
    for (int i = 0; i < 3; ++i) {
      position_[static_cast<size_t>(i)].store(position(i),
                                              std::memory_order_relaxed);
    }
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        rotation_[static_cast<size_t>(3 * row + col)].store(
            rotation(row, col), std::memory_order_relaxed);
      }
    }
    sequence_.fetch_add(1, std::memory_order_release);
    valid_.store(true, std::memory_order_release);
  }

  bool read(TcpPose &pose) const noexcept {
    if (!valid_.load(std::memory_order_acquire))
      return false;
    for (int attempt = 0; attempt < 3; ++attempt) {
      const std::uint64_t before = sequence_.load(std::memory_order_acquire);
      if ((before & 1U) != 0U)
        continue;
      for (int i = 0; i < 3; ++i) {
        pose.position(i) =
            position_[static_cast<size_t>(i)].load(std::memory_order_relaxed);
      }
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
          pose.rotation(row, col) =
              rotation_[static_cast<size_t>(3 * row + col)].load(
                  std::memory_order_relaxed);
        }
      }
      const std::uint64_t after = sequence_.load(std::memory_order_acquire);
      if (before == after && (after & 1U) == 0U)
        return true;
    }
    return false;
  }

private:
  std::array<std::atomic<double>, 3> position_{};
  std::array<std::atomic<double>, 9> rotation_{};
  std::atomic<std::uint64_t> sequence_{0};
  std::atomic<bool> valid_{false};
};

class AtomicJointPositions {
public:
  void write(const std::array<double, 7> &positions) noexcept {
    sequence_.fetch_add(1, std::memory_order_acq_rel);
    for (size_t i = 0; i < positions.size(); ++i) {
      positions_[i].store(positions[i], std::memory_order_relaxed);
    }
    sequence_.fetch_add(1, std::memory_order_release);
    valid_.store(true, std::memory_order_release);
  }

  bool read(std::array<double, 7> &positions) const noexcept {
    if (!valid_.load(std::memory_order_acquire))
      return false;
    for (int attempt = 0; attempt < 3; ++attempt) {
      const std::uint64_t before = sequence_.load(std::memory_order_acquire);
      if ((before & 1U) != 0U)
        continue;
      for (size_t i = 0; i < positions.size(); ++i) {
        positions[i] = positions_[i].load(std::memory_order_relaxed);
      }
      const std::uint64_t after = sequence_.load(std::memory_order_acquire);
      if (before == after && (after & 1U) == 0U)
        return true;
    }
    return false;
  }

private:
  std::array<std::atomic<double>, 7> positions_{};
  std::atomic<std::uint64_t> sequence_{0};
  std::atomic<bool> valid_{false};
};

geometry_msgs::msg::TransformStamped makeTransform(const rclcpp::Time &stamp,
                                                   const std::string &parent,
                                                   const std::string &child,
                                                   const TcpPose &pose) {
  Eigen::Quaterniond quaternion(pose.rotation);
  quaternion.normalize();

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = parent;
  transform.child_frame_id = child;
  transform.transform.translation.x = pose.position.x();
  transform.transform.translation.y = pose.position.y();
  transform.transform.translation.z = pose.position.z();
  transform.transform.rotation.x = quaternion.x();
  transform.transform.rotation.y = quaternion.y();
  transform.transform.rotation.z = quaternion.z();
  transform.transform.rotation.w = quaternion.w();
  return transform;
}

void printHelp(const char *program) {
  std::cout
      << "Usage: " << program << " [options]\n\n"
      << "Hold the startup TCP pose or ramp to a YAML target with admittance "
         "active, and publish "
         "TCP frames for RViz. Without --execute this is a dry run.\n\n"
      << "Options:\n"
      << "  --config <yaml>             Controller YAML (default: "
      << TCP_ADMITTANCE_DEFAULT_CONFIG << ")\n"
      << "  --robot-hostname <ip>       Override robot.hostname\n"
      << "  --profile <name>            Override default_profile\n"
      << "  --base-frame <frame>        TF parent / RViz fixed frame (default: "
         "fr3_link0)\n"
      << "  --actual-tcp-frame <frame>  Measured TCP child frame (default: "
         "tcp_actual)\n"
      << "  --command-tcp-frame <frame> Compliant command child frame "
         "(default: tcp_command)\n"
      << "  --home-tcp-frame <frame>    Fixed hold target child frame "
         "(default: tcp_home)\n"
      << "  --tf-rate <hz>              TF publication rate (default: 30)\n"
      << "  --execute                   Connect to and control the real robot\n"
      << "  --help                      Show this text\n";
}

void printPlan(const CliArgs &cli, const CartesianAdmittanceConfig &config,
               const std::string &profile) {
  std::cout << "\nTCP admittance home-hold plan\n"
            << "  Config: " << cli.config_path << "\n"
            << "  Robot: " << config.robot_hostname << "\n"
            << "  Profile: " << (profile.empty() ? "none" : profile) << "\n";
  if (config.hold_initial_tcp_pose) {
    std::cout << "  Hold target: physical TCP pose captured at controller "
                 "startup\n";
  } else {
    const Eigen::Vector3d target_rpy =
        my_controller::rotationMatrixToRPY(*config.target_rotation_world);
    std::cout << "  Target TCP position in O [m]: ["
              << config.target_position_world.transpose() << "]\n"
              << "  Target TCP RPY in O [rad]: [" << target_rpy.transpose()
              << "]\n";
  }
  std::cout << "  Ramp time: " << config.target_ramp_time << " s\n"
            << "  Wrench: " << config.wrench_source << " / "
            << config.wrench_frame << "\n"
            << "  Compliance frame: " << config.admittance_frame << "\n"
            << "  Run time: until Ctrl+C\n"
            << "  RViz Fixed Frame: " << cli.base_frame << "\n"
            << "  TF children: " << cli.actual_tcp_frame << ", "
            << cli.command_tcp_frame << ", " << cli.home_tcp_frame << "\n"
            << "  URDF joint states: /joint_states at " << cli.tf_rate
            << " Hz\n"
            << "  Pose log directory: " << config.pose_log_dir << "\n";
}

} // namespace

int main(int argc, char **argv) {
  try {
    const CliArgs cli = parseCli(argc, argv);
    if (cli.show_help) {
      printHelp(argv[0]);
      return 0;
    }

    CartesianAdmittanceConfig config;
    const std::string profile =
        loadConfig(cli.config_path, cli.profile, config);
    if (cli.robot_hostname)
      config.robot_hostname = *cli.robot_hostname;
    validateConfig(config);
    printPlan(cli, config, profile);

    if (!cli.execute) {
      std::cout << "Dry run only: robot control and TF publication were NOT "
                   "started.\n";
      return 0;
    }
    if (!isatty(STDIN_FILENO)) {
      throw std::runtime_error("--execute requires an interactive terminal.");
    }

    std::cout
        << "WARNING: this directly controls the real robot through libfranka.\n"
        << "Stop other robot controllers, clear the workspace, keep the TCP "
           "unloaded during "
           "startup/ramp, keep an E-stop ready, and press Enter to continue: "
        << std::flush;
    std::string confirmation;
    if (!std::getline(std::cin, confirmation)) {
      std::cout << "Confirmation not received; controller was NOT called.\n";
      return 0;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("tcp_admittance_home_hold");
    tf2_ros::TransformBroadcaster broadcaster(node);
    auto joint_state_publisher =
        node->create_publisher<sensor_msgs::msg::JointState>("joint_states",
                                                             10);
    AtomicTcpPose actual_pose;
    AtomicTcpPose command_pose;
    AtomicTcpPose hold_pose;
    AtomicJointPositions joint_positions;
    const bool capture_startup_target = config.hold_initial_tcp_pose;
    std::atomic<bool> startup_target_captured{false};
    if (!capture_startup_target) {
      hold_pose.write(config.target_position_world,
                      *config.target_rotation_world);
    }
    config.tcp_pose_observer =
        [&actual_pose, &command_pose, &hold_pose, &startup_target_captured,
         capture_startup_target](const Eigen::Vector3d &actual_position,
                                 const Eigen::Matrix3d &actual_rotation,
                                 const Eigen::Vector3d &command_position,
                                 const Eigen::Matrix3d &command_rotation) {
          if (capture_startup_target && !startup_target_captured.exchange(
                                            true, std::memory_order_acq_rel)) {
            hold_pose.write(actual_position, actual_rotation);
          }
          actual_pose.write(actual_position, actual_rotation);
          command_pose.write(command_position, command_rotation);
        };
    config.joint_position_observer =
        [&joint_positions](const std::array<double, 7> &positions) {
          joint_positions.write(positions);
        };

    std::atomic<bool> stop_tf{false};
    std::thread tf_thread([&]() {
      const auto period = std::chrono::duration<double>(1.0 / cli.tf_rate);
      sensor_msgs::msg::JointState joint_state;
      joint_state.name = {"fr3_joint1", "fr3_joint2", "fr3_joint3",
                          "fr3_joint4", "fr3_joint5", "fr3_joint6",
                          "fr3_joint7"};
      joint_state.position.resize(7);
      while (!stop_tf.load(std::memory_order_acquire) && rclcpp::ok()) {
        const rclcpp::Time stamp = node->now();
        TcpPose pose;
        if (hold_pose.read(pose)) {
          broadcaster.sendTransform(
              makeTransform(stamp, cli.base_frame, cli.home_tcp_frame, pose));
        }
        if (actual_pose.read(pose)) {
          broadcaster.sendTransform(
              makeTransform(stamp, cli.base_frame, cli.actual_tcp_frame, pose));
        }
        if (command_pose.read(pose)) {
          broadcaster.sendTransform(makeTransform(stamp, cli.base_frame,
                                                  cli.command_tcp_frame, pose));
        }
        std::array<double, 7> positions{};
        if (joint_positions.read(positions)) {
          joint_state.header.stamp = stamp;
          for (size_t i = 0; i < positions.size(); ++i) {
            joint_state.position[i] = positions[i];
          }
          joint_state_publisher->publish(joint_state);
        }
        std::this_thread::sleep_for(period);
      }
    });

    const bool success = my_controller::runCartesianAdmittanceToTarget(config);
    stop_tf.store(true, std::memory_order_release);
    tf_thread.join();
    if (rclcpp::ok())
      rclcpp::shutdown();

    std::cout << (success ? "TCP home-hold controller stopped normally.\n"
                          : "TCP home-hold controller failed; inspect the "
                            "errors above.\n");
    return success ? 0 : 1;
  } catch (const YAML::Exception &error) {
    std::cerr << "YAML configuration error: " << error.what() << '\n';
    return 2;
  } catch (const std::exception &error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 2;
  }
}
