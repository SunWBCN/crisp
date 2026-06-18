#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace my_controller {

using PoseMatrix = std::array<std::array<double, 4>, 4>;

struct CartesianImpedanceGains {
  // Order: x, y, z, rx, ry, rz.
  std::array<double, 6> stiffness;
  std::array<double, 6> damping;
};

CartesianImpedanceGains make_cartesian_impedance_gains(
  double position_stiffness,
  double rotation_stiffness,
  double position_damping,
  double rotation_damping);

CartesianImpedanceGains make_cartesian_impedance_gains(
  const std::array<double, 6> & stiffness,
  const std::array<double, 6> & damping);

geometry_msgs::msg::PoseStamped make_target_pose(
  const std::array<double, 3> & position,
  const std::array<double, 4> & orientation_xyzw,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

geometry_msgs::msg::PoseStamped make_target_pose(
  const PoseMatrix & target_pose_matrix,
  const std::string & frame_id,
  const rclcpp::Time & stamp);

class ImpedanceCommandClient {
public:
  explicit ImpedanceCommandClient(
    rclcpp::Node::SharedPtr node,
    std::string controller_name = "cartesian_impedance_controller",
    std::string target_pose_topic = "target_pose",
    std::string base_frame = "base");

  [[nodiscard]] bool wait_until_ready(
    std::chrono::nanoseconds timeout = std::chrono::seconds(5)) const;

  [[nodiscard]] bool set_gains(
    const CartesianImpedanceGains & gains,
    std::chrono::nanoseconds timeout = std::chrono::seconds(5)) const;

  [[nodiscard]] bool switch_to_controller(
    std::chrono::nanoseconds timeout = std::chrono::seconds(5)) const;

  [[nodiscard]] std::optional<geometry_msgs::msg::PoseStamped> wait_for_current_pose(
    const std::string & current_pose_topic = "current_pose",
    std::chrono::nanoseconds timeout = std::chrono::seconds(5)) const;

  void publish_target_pose(const geometry_msgs::msg::PoseStamped & target_pose) const;

  [[nodiscard]] bool command(
    const geometry_msgs::msg::PoseStamped & target_pose,
    const CartesianImpedanceGains & gains,
    std::chrono::nanoseconds timeout = std::chrono::seconds(5)) const;

  [[nodiscard]] geometry_msgs::msg::PoseStamped make_target_pose(
    const std::array<double, 3> & position,
    const std::array<double, 4> & orientation_xyzw) const;

  [[nodiscard]] geometry_msgs::msg::PoseStamped make_target_pose(
    const PoseMatrix & target_pose_matrix) const;

  [[nodiscard]] geometry_msgs::msg::PoseStamped make_relative_target_pose(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const std::array<double, 3> & position_offset,
    const std::array<double, 3> & rpy_offset) const;

  [[nodiscard]] bool command_relative_to_current(
    const CartesianImpedanceGains & gains,
    const std::array<double, 3> & position_offset,
    const std::array<double, 3> & rpy_offset,
    const std::string & current_pose_topic = "current_pose",
    double publish_seconds = 3.0,
    double publish_rate_hz = 50.0,
    std::chrono::nanoseconds timeout = std::chrono::seconds(5)) const;

  [[nodiscard]] bool command_target_pose(
    const CartesianImpedanceGains & gains,
    const std::array<double, 3> & target_position,
    const std::array<double, 4> & target_orientation_xyzw,
    double publish_seconds = 3.0,
    double publish_rate_hz = 50.0,
    std::chrono::nanoseconds timeout = std::chrono::seconds(5)) const;

  [[nodiscard]] bool command_target_pose(
    const CartesianImpedanceGains & gains,
    const PoseMatrix & target_pose_matrix,
    double publish_seconds = 3.0,
    double publish_rate_hz = 50.0,
    std::chrono::nanoseconds timeout = std::chrono::seconds(5)) const;

private:
  rclcpp::Node::SharedPtr node_;
  std::string controller_name_;
  std::string base_frame_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client_;
};

[[nodiscard]] bool command_relative_impedance(
  rclcpp::Node::SharedPtr node,
  const CartesianImpedanceGains & gains,
  const std::array<double, 3> & position_offset,
  const std::array<double, 3> & rpy_offset,
  const std::string & controller_name = "cartesian_impedance_controller",
  const std::string & current_pose_topic = "current_pose",
  const std::string & target_pose_topic = "target_pose",
  double publish_seconds = 3.0,
  double publish_rate_hz = 50.0,
  std::chrono::nanoseconds timeout = std::chrono::seconds(5));

[[nodiscard]] bool command_impedance_target_pose(
  rclcpp::Node::SharedPtr node,
  const CartesianImpedanceGains & gains,
  const std::array<double, 3> & target_position,
  const std::array<double, 4> & target_orientation_xyzw,
  const std::string & controller_name = "cartesian_impedance_controller",
  const std::string & target_pose_topic = "target_pose",
  double publish_seconds = 3.0,
  double publish_rate_hz = 50.0,
  std::chrono::nanoseconds timeout = std::chrono::seconds(5));

[[nodiscard]] bool command_impedance_target_matrix(
  rclcpp::Node::SharedPtr node,
  const CartesianImpedanceGains & gains,
  const PoseMatrix & target_pose_matrix,
  const std::string & controller_name = "cartesian_impedance_controller",
  const std::string & target_pose_topic = "target_pose",
  double publish_seconds = 3.0,
  double publish_rate_hz = 50.0,
  std::chrono::nanoseconds timeout = std::chrono::seconds(5));

[[nodiscard]] bool command_impedance_target_matrix(
  rclcpp::Node::SharedPtr node,
  const std::array<double, 6> & stiffness,
  const std::array<double, 6> & damping,
  const PoseMatrix & target_pose_matrix,
  const std::string & controller_name = "cartesian_impedance_controller",
  const std::string & target_pose_topic = "target_pose",
  double publish_seconds = 3.0,
  double publish_rate_hz = 50.0,
  std::chrono::nanoseconds timeout = std::chrono::seconds(5));

}  // namespace my_controller
