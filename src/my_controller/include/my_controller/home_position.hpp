#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace my_controller {

[[nodiscard]] std::vector<std::string> franka_joint_names();
[[nodiscard]] std::vector<double> franka_home_joint_positions();

[[nodiscard]] bool command_home_position(
  rclcpp::Node::SharedPtr node,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & joint_positions,
  const std::string & controller_name = "joint_trajectory_controller",
  double move_duration_seconds = 5.0,
  std::chrono::nanoseconds timeout = std::chrono::seconds(10));

[[nodiscard]] bool command_franka_home_position(
  rclcpp::Node::SharedPtr node,
  const std::string & controller_name = "joint_trajectory_controller",
  double move_duration_seconds = 5.0,
  std::chrono::nanoseconds timeout = std::chrono::seconds(10));

}  // namespace my_controller
