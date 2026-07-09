// Thin terminal wrapper around runCartesianAdmittanceToTarget(). Almost all tuning now
// lives in a YAML config file next to this source file (cartesian_admittance.yaml); the wrapper only
// loads that file into a CartesianAdmittanceConfig and runs the reusable controller. The
// target is always a full world-frame TCP pose (O_T_TCP_target), never a flange pose; the
// controller converts it back to the flange command with the existing TCP offset.
//
// All file I/O, YAML parsing, logging and terminal printing happen here, before the
// controller starts -- nothing in this file runs inside the 1 kHz real-time callback.
//
// Build:
//   pixi run -e jazzy-crisp colcon build --packages-select my_controller --symlink-install
//
// Run (config path is optional; it defaults to the repo config below):
//   pixi run -e jazzy-crisp ros2 run my_controller cartesian_admittance 10.90.90.10
//     --config /home/wenbin/github_repo/crisp_franka/src/my_controller/scripts/cartesian_admittance.yaml
//   pixi run -e jazzy-crisp ros2 run my_controller cartesian_admittance --help
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>

#include "my_controller/cartesian_admittance.hpp"

namespace {

using my_controller::CartesianAdmittanceConfig;

// Default YAML config, loaded when --config is not given.
constexpr const char * kDefaultConfigPath =
  "/home/wenbin/github_repo/crisp_franka/src/my_controller/scripts/cartesian_admittance.yaml";

// ---------------------------------------------------------------------------
// YAML value helpers. Each one produces a clear std::invalid_argument that names
// the offending key so a bad config file is easy to fix.
// ---------------------------------------------------------------------------

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

// Describes where the target TCP pose came from, purely for the startup summary.
enum class TargetSource { kDefault, kManualPose, kHome };

// Result of loading a config file, used only for the startup summary.
struct LoadResult {
  TargetSource target_source = TargetSource::kDefault;
  std::string profile;   // admittance preset actually applied ("" if none)
};

// Applies the admittance gains found in `node` (any of mask/mass/stiffness/damping).
// Shared by the top-level `admittance:` block and by each named profile.
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

// Loads the YAML file into config. Every key is optional: a missing key keeps the
// CartesianAdmittanceConfig default, a present key overrides it. Values are validated as
// they are read; structural rules (e.g. only one target) are enforced here too. The
// admittance stiffness/damping come from the selected profile: requested_profile if given,
// otherwise the file's default_profile.
LoadResult loadConfigFile(
  const std::string & path, const std::optional<std::string> & requested_profile,
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

  TargetSource target_source = TargetSource::kDefault;
  if (const YAML::Node target = root["target"]) {
    const YAML::Node pose = target["pose"];
    const YAML::Node pose_source = target["pose_source"];
    if (pose && pose_source) {
      throw std::invalid_argument(
        "target: use only one of 'pose' or 'pose_source' (not both).");
    }
    if (pose) {
      const std::array<double, 6> p = asVector6(pose, "target.pose");
      config.target_position_world = Eigen::Vector3d(p[0], p[1], p[2]);
      config.target_rotation_world = my_controller::rpyToRotationMatrix(p[3], p[4], p[5]);
      target_source = TargetSource::kManualPose;
    }
    if (pose_source) {
      const std::string src = pose_source.as<std::string>();
      if (src != "home") {
        throw std::invalid_argument(
          "target.pose_source: only 'home' is supported (or set target.pose instead).");
      }
      config.target_position_world = my_controller::homeTcpPosition();
      config.target_rotation_world = my_controller::homeTcpRotation();
      target_source = TargetSource::kHome;
    }
    if (const YAML::Node v = target["ramp_time"]) {
      config.target_ramp_time = asDouble(v, "target.ramp_time");
    }
  }

  if (const YAML::Node adm = root["admittance"]) {
    applyAdmittance(adm, config, "admittance");
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

  if (const YAML::Node run = root["run"]) {
    if (const YAML::Node v = run["run_time"]) config.run_time = asDouble(v, "run.run_time");
    if (const YAML::Node v = run["stop_on_settled"]) {
      config.stop_on_settled = asBool(v, "run.stop_on_settled");
    }
    if (const YAML::Node v = run["settle_window"]) {
      config.settle_window = asDouble(v, "run.settle_window");
    }
    if (const YAML::Node v = run["settle_position_threshold"]) {
      config.settle_position_threshold = asDouble(v, "run.settle_position_threshold");
    }
    if (const YAML::Node v = run["settle_rotation_threshold"]) {
      config.settle_rotation_threshold = asDouble(v, "run.settle_rotation_threshold");
    }
    if (const YAML::Node v = run["settle_velocity_threshold"]) {
      config.settle_velocity_threshold = asDouble(v, "run.settle_velocity_threshold");
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
  }

  // Select and apply the admittance preset: --profile if given, else default_profile.
  std::string applied_profile;
  std::optional<std::string> selected = requested_profile;
  if (!selected) {
    if (const YAML::Node dp = root["default_profile"]) selected = dp.as<std::string>();
  }
  if (selected && !selected->empty()) {
    const YAML::Node profiles = root["profiles"];
    if (!profiles || !profiles.IsMap()) {
      throw std::invalid_argument(
        "Profile '" + *selected + "' requested but the config has no 'profiles:' section.");
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
    applied_profile = *selected;
  }

  return LoadResult{target_source, applied_profile};
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
}

// ---------------------------------------------------------------------------
// CLI: only a leading hostname, --robot-hostname, --config and --help remain.
// Everything else is configured through the YAML file.
// ---------------------------------------------------------------------------

struct CliArgs {
  bool show_help = false;
  std::string config_path = kDefaultConfigPath;
  std::optional<std::string> robot_hostname;   // explicit override, applied after YAML load
  std::optional<std::string> profile;          // admittance preset override (e.g. soft/hard)
};

CliArgs parseCli(const std::vector<std::string> & args) {
  CliArgs cli;
  size_t i = 1;
  // Optional leading positional robot hostname (anything not starting with '-').
  if (i < args.size() && !args[i].empty() && args[i].front() != '-') {
    cli.robot_hostname = args[i++];
  }
  for (; i < args.size(); ++i) {
    const std::string & opt = args[i];
    if (opt == "--help" || opt == "-h") {
      cli.show_help = true;
    } else if (opt == "--config") {
      if (i + 1 >= args.size()) throw std::invalid_argument("--config requires a value.");
      cli.config_path = args[++i];
    } else if (opt == "--robot-hostname") {
      if (i + 1 >= args.size()) throw std::invalid_argument("--robot-hostname requires a value.");
      cli.robot_hostname = args[++i];
    } else if (opt == "--profile") {
      if (i + 1 >= args.size()) throw std::invalid_argument("--profile requires a value.");
      cli.profile = args[++i];
    } else {
      throw std::invalid_argument(
        "Unknown option '" + opt + "'. Most parameters now live in the YAML config; "
        "use --help for the short option list.");
    }
  }
  return cli;
}

void printHelp(const std::string & program) {
  std::cout <<
    "Usage: " << program
              << " [robot-hostname] [--config <path>] [--profile <name>] [--robot-hostname <ip>] [--help]\n"
    "\n"
    "Cartesian admittance controller (libfranka). Ramps the TCP to a target pose in the\n"
    "world/base frame with a minimum-jerk profile, then holds it while staying compliant.\n"
    "The target is always a full TCP pose (O_T_TCP_target), never a flange pose.\n"
    "\n"
    "Almost every parameter -- wrench source, target pose, admittance gains, safety\n"
    "limits, settling detector and logging/plotting -- now comes from a YAML config file.\n"
    "Edit that file instead of passing long command lines.\n"
    "\n"
    "Options:\n"
    "  [robot-hostname]       Optional leading robot IP; overrides robot.hostname in YAML.\n"
    "  --robot-hostname <ip>  Same as the leading positional hostname.\n"
    "  --config <path>        YAML config file to load.\n"
    "                         (default: " << kDefaultConfigPath << ")\n"
    "  --profile <name>       Admittance preset from the YAML 'profiles:' section, e.g.\n"
    "                         'soft' or 'hard'. Overrides the file's default_profile.\n"
    "  --help, -h             Show this help and exit.\n"
    "\n"
    "Example:\n"
    "  " << program << " 10.90.90.10 \\\n"
    "    --config " << kDefaultConfigPath << "\n"
    "\n"
    "See the YAML file for the full list of tunable parameters and their meaning.\n"
    << std::endl;
}

void printSummary(
  const CartesianAdmittanceConfig & config, const std::string & config_path,
  TargetSource target_source, const std::string & profile) {
  const char * target_desc =
    target_source == TargetSource::kHome ? "predefined home TCP pose"
    : target_source == TargetSource::kManualPose ? "manual TCP pose (target.pose)"
    : "default (no target given in YAML)";

  std::cout << "WARNING: This program directly controls the robot through libfranka.\n"
            << "Stop any ROS2 franka hardware/controller_manager launch before running it.\n"
            << "Franka error recovery will be requested before starting control.\n"
            << "Config: " << config_path << "\n"
            << "Profile: " << (profile.empty() ? "none" : profile) << "\n"
            << "Robot: " << config.robot_hostname << "\n"
            << "Wrench source: " << config.wrench_source << " (frame " << config.wrench_frame
            << ", sign " << config.wrench_sign << ")\n";
  if (config.wrench_source == "serial") {
    std::cout << "Serial port: " << config.serial_port << "\n";
  }
  if (config.wrench_source == "topic") {
    std::cout << "Wrench topic: " << config.wrench_topic << "\n";
  }
  std::cout << "Target source: " << target_desc << "\n"
            << "Target TCP position (world) [m]: [" << config.target_position_world.x() << ", "
            << config.target_position_world.y() << ", " << config.target_position_world.z()
            << "]\n";
  if (config.target_rotation_world) {
    const Eigen::Vector3d rpy = my_controller::rotationMatrixToRPY(*config.target_rotation_world);
    std::cout << "Target TCP orientation (world RPY) [rad]: [" << rpy.x() << ", " << rpy.y() << ", "
              << rpy.z() << "]\n";
  } else {
    std::cout << "Target TCP orientation: keep the initial measured TCP orientation\n";
  }

  const auto printVec = [](const char * label, const std::array<double, 6> & v) {
    std::cout << label << ": [" << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << " "
              << v[4] << " " << v[5] << "]\n";
  };
  printVec("Mask [Fx Fy Fz Mx My Mz]", config.admittance_mask);
  printVec("Mass [Fx Fy Fz Mx My Mz]", config.admittance_mass);
  printVec("Stiffness [Fx Fy Fz Mx My Mz]", config.admittance_stiffness);
  printVec("Damping [Fx Fy Fz Mx My Mz]", config.admittance_damping);

  std::cout << "Ramp " << config.target_ramp_time << " s, run-time " << config.run_time
            << " s, stop-on-settled " << (config.stop_on_settled ? "true" : "false") << "\n"
            << "CSV log-pose-error " << (config.log_pose_error ? "true" : "false")
            << " | pose-log-dir " << config.pose_log_dir
            << " | auto-plot " << (config.auto_plot ? "true" : "false")
            << " | print-pose-error " << (config.print_pose_error ? "true" : "false")
            << " | print-wrench-debug " << (config.print_wrench_debug ? "true" : "false")
            << " | print-period " << config.print_period << " s\n"
            << "Press Enter to start..." << std::endl;
}

}  // namespace

int main(int argc, char ** argv) {
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  const std::string program = args.empty() ? "cartesian_admittance" : args[0];

  // 1. Parse only --config, --help and the optional robot hostname.
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
  LoadResult loaded;
  try {
    // 2. Load the YAML file into the config (applying the selected admittance profile).
    loaded = loadConfigFile(cli.config_path, cli.profile, config);
    // 3. Apply explicit CLI overrides (currently just the robot hostname).
    if (cli.robot_hostname) {
      config.robot_hostname = *cli.robot_hostname;
    }
    // 4. Validate the resulting config.
    validateConfig(config);
  } catch (const std::exception & ex) {
    std::cerr << "Configuration error: " << ex.what() << std::endl;
    return 1;
  }

  // 5. Print a short summary.
  printSummary(config, cli.config_path, loaded.target_source, loaded.profile);

  // 6. Wait for Enter.
  if (!isatty(STDIN_FILENO)) {
    std::cerr << "No interactive terminal was detected; not starting control." << std::endl;
    return 1;
  }
  std::cin.ignore();
  if (!std::cin) {
    std::cerr << "No Enter key was received; not starting control." << std::endl;
    return 1;
  }

  // 7. Run the controller (all file I/O and parsing above happen before this point).
  my_controller::runCartesianAdmittanceToTarget(config);
  return 0;
}
