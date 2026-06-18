#include <my_controller/impedance_controller.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <crisp_controllers/cartesian_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace my_controller {

namespace {

constexpr std::array<const char *, 6> kStiffnessParameters = {
  "task.k_pos_x", "task.k_pos_y", "task.k_pos_z",
  "task.k_rot_x", "task.k_rot_y", "task.k_rot_z"};

constexpr std::array<const char *, 6> kDampingParameters = {
  "task.d_pos_x", "task.d_pos_y", "task.d_pos_z",
  "task.d_rot_x", "task.d_rot_y", "task.d_rot_z"};

bool ends_with(const std::string & value, const std::string & suffix) {
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

void validate_gains(const CartesianImpedanceGains & gains) {
  for (size_t i = 0; i < gains.stiffness.size(); ++i) {
    if (!std::isfinite(gains.stiffness[i]) || gains.stiffness[i] < 0.0) {
      throw std::invalid_argument("Impedance stiffness K must be finite and non-negative.");
    }
    if (!std::isfinite(gains.damping[i]) || gains.damping[i] < -1.0) {
      throw std::invalid_argument(
        "Impedance damping D must be finite and >= -1.0. Use -1.0 for CRISP auto damping.");
    }
  }
}

void validate_pose_matrix(const PoseMatrix & target_pose_matrix) {
  for (const auto & row : target_pose_matrix) {
    for (const auto value : row) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("Target pose matrix must contain only finite values.");
      }
    }
  }

  constexpr double kTolerance = 1e-9;
  if (std::abs(target_pose_matrix[3][0]) > kTolerance ||
    std::abs(target_pose_matrix[3][1]) > kTolerance ||
    std::abs(target_pose_matrix[3][2]) > kTolerance ||
    std::abs(target_pose_matrix[3][3] - 1.0) > kTolerance)
  {
    throw std::invalid_argument("Target pose matrix last row must be [0, 0, 0, 1].");
  }
}

template <typename FutureT>
bool wait_for_future(
  const rclcpp::Node::SharedPtr & node,
  FutureT & future,
  std::chrono::nanoseconds timeout) {
  try {
    const auto result = rclcpp::spin_until_future_complete(node, future, timeout);
    return result == rclcpp::FutureReturnCode::SUCCESS;
  } catch (const std::runtime_error &) {
    return future.wait_for(timeout) == std::future_status::ready;
  }
}

bool wait_for_set_parameters(
  const rclcpp::Node::SharedPtr & node,
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future,
  std::chrono::nanoseconds timeout) {
  if (!wait_for_future(node, future, timeout)) {
    return false;
  }

  for (const auto & parameter_result : future.get()) {
    if (!parameter_result.successful) {
      return false;
    }
  }
  return true;
}

}  // namespace

CartesianImpedanceGains make_cartesian_impedance_gains(
  double position_stiffness,
  double rotation_stiffness,
  double position_damping,
  double rotation_damping) {
  CartesianImpedanceGains gains{
    {position_stiffness, position_stiffness, position_stiffness,
      rotation_stiffness, rotation_stiffness, rotation_stiffness},
    {position_damping, position_damping, position_damping,
      rotation_damping, rotation_damping, rotation_damping}};
  validate_gains(gains);
  return gains;
}

CartesianImpedanceGains make_cartesian_impedance_gains(
  const std::array<double, 6> & stiffness,
  const std::array<double, 6> & damping) {
  CartesianImpedanceGains gains{stiffness, damping};
  validate_gains(gains);
  return gains;
}

geometry_msgs::msg::PoseStamped make_target_pose(
  const std::array<double, 3> & position,
  const std::array<double, 4> & orientation_xyzw,
  const std::string & frame_id,
  const rclcpp::Time & stamp) {
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = frame_id;
  target_pose.header.stamp = stamp;
  target_pose.pose.position.x = position[0];
  target_pose.pose.position.y = position[1];
  target_pose.pose.position.z = position[2];
  target_pose.pose.orientation.x = orientation_xyzw[0];
  target_pose.pose.orientation.y = orientation_xyzw[1];
  target_pose.pose.orientation.z = orientation_xyzw[2];
  target_pose.pose.orientation.w = orientation_xyzw[3];
  return target_pose;
}

geometry_msgs::msg::PoseStamped make_target_pose(
  const PoseMatrix & target_pose_matrix,
  const std::string & frame_id,
  const rclcpp::Time & stamp) {
  validate_pose_matrix(target_pose_matrix);

  tf2::Matrix3x3 rotation(
    target_pose_matrix[0][0], target_pose_matrix[0][1], target_pose_matrix[0][2],
    target_pose_matrix[1][0], target_pose_matrix[1][1], target_pose_matrix[1][2],
    target_pose_matrix[2][0], target_pose_matrix[2][1], target_pose_matrix[2][2]);

  tf2::Quaternion orientation;
  rotation.getRotation(orientation);
  orientation.normalize();

  return make_target_pose(
    {target_pose_matrix[0][3], target_pose_matrix[1][3], target_pose_matrix[2][3]},
    {orientation.x(), orientation.y(), orientation.z(), orientation.w()},
    frame_id,
    stamp);
}

ImpedanceCommandClient::ImpedanceCommandClient(
  rclcpp::Node::SharedPtr node,
  std::string controller_name,
  std::string target_pose_topic,
  std::string base_frame)
: node_(std::move(node)),
  controller_name_(std::move(controller_name)),
  base_frame_(std::move(base_frame)) {
  target_pose_pub_ =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>(std::move(target_pose_topic), 10);
  parameters_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(node_, controller_name_);
}

bool ImpedanceCommandClient::wait_until_ready(std::chrono::nanoseconds timeout) const {
  return parameters_client_->wait_for_service(timeout);
}

bool ImpedanceCommandClient::set_gains(
  const CartesianImpedanceGains & gains,
  std::chrono::nanoseconds timeout) const {
  validate_gains(gains);

  std::vector<rclcpp::Parameter> parameters;
  parameters.reserve(kStiffnessParameters.size() + kDampingParameters.size());

  for (size_t i = 0; i < kStiffnessParameters.size(); ++i) {
    parameters.emplace_back(kStiffnessParameters[i], gains.stiffness[i]);
  }
  for (size_t i = 0; i < kDampingParameters.size(); ++i) {
    parameters.emplace_back(kDampingParameters[i], gains.damping[i]);
  }

  return wait_for_set_parameters(node_, parameters_client_->set_parameters(parameters), timeout);
}

bool ImpedanceCommandClient::switch_to_controller(std::chrono::nanoseconds timeout) const {
  auto list_client =
    node_->create_client<controller_manager_msgs::srv::ListControllers>(
      "controller_manager/list_controllers");
  auto switch_client =
    node_->create_client<controller_manager_msgs::srv::SwitchController>(
      "controller_manager/switch_controller");

  if (!list_client->wait_for_service(timeout) || !switch_client->wait_for_service(timeout)) {
    RCLCPP_ERROR(node_->get_logger(), "controller_manager services are not available.");
    return false;
  }

  auto list_future =
    list_client->async_send_request(
      std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
  if (!wait_for_future(node_, list_future, timeout)) {
    RCLCPP_ERROR(node_->get_logger(), "Timed out while listing controllers.");
    return false;
  }

  const auto controllers = list_future.get()->controller;
  const auto target_controller =
    std::find_if(controllers.begin(), controllers.end(), [this](const auto & controller) {
      return controller.name == controller_name_;
    });
  if (target_controller == controllers.end()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Controller '%s' is not loaded. Start the Franka launch file first.",
      controller_name_.c_str());
    return false;
  }

  if (target_controller->state == "active") {
    return true;
  }

  std::vector<std::string> deactivate_controllers;
  for (const auto & controller : controllers) {
    if (controller.state == "active" && !ends_with(controller.name, "broadcaster")) {
      deactivate_controllers.push_back(controller.name);
    }
  }

  auto request =
    std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->deactivate_controllers = deactivate_controllers;
  request->activate_controllers = {controller_name_};
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  request->activate_asap = true;
  request->timeout.sec = 5;
  request->timeout.nanosec = 0;

  auto switch_future = switch_client->async_send_request(request);
  if (!wait_for_future(node_, switch_future, timeout)) {
    RCLCPP_ERROR(node_->get_logger(), "Timed out while switching controllers.");
    return false;
  }

  if (!switch_future.get()->ok) {
    RCLCPP_ERROR(
      node_->get_logger(), "Failed to switch to controller '%s'.", controller_name_.c_str());
    return false;
  }

  return true;
}

std::optional<geometry_msgs::msg::PoseStamped> ImpedanceCommandClient::wait_for_current_pose(
  const std::string & current_pose_topic,
  std::chrono::nanoseconds timeout) const {
  std::optional<geometry_msgs::msg::PoseStamped> current_pose;

  auto sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    current_pose_topic, 10,
    [&current_pose](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      current_pose = *msg;
    });

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  rclcpp::WallRate rate(100.0);
  while (rclcpp::ok() && !current_pose && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node_);
    rate.sleep();
  }

  return current_pose;
}

void ImpedanceCommandClient::publish_target_pose(
  const geometry_msgs::msg::PoseStamped & target_pose) const {
  target_pose_pub_->publish(target_pose);
}

bool ImpedanceCommandClient::command(
  const geometry_msgs::msg::PoseStamped & target_pose,
  const CartesianImpedanceGains & gains,
  std::chrono::nanoseconds timeout) const {
  if (!set_gains(gains, timeout)) {
    return false;
  }
  publish_target_pose(target_pose);
  return true;
}

geometry_msgs::msg::PoseStamped ImpedanceCommandClient::make_target_pose(
  const std::array<double, 3> & position,
  const std::array<double, 4> & orientation_xyzw) const {
  return my_controller::make_target_pose(position, orientation_xyzw, base_frame_, node_->now());
}

geometry_msgs::msg::PoseStamped ImpedanceCommandClient::make_target_pose(
  const PoseMatrix & target_pose_matrix) const {
  return my_controller::make_target_pose(target_pose_matrix, base_frame_, node_->now());
}

geometry_msgs::msg::PoseStamped ImpedanceCommandClient::make_relative_target_pose(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const std::array<double, 3> & position_offset,
  const std::array<double, 3> & rpy_offset) const {
  auto target_pose = current_pose;
  target_pose.header.stamp = node_->now();
  target_pose.pose.position.x += position_offset[0];
  target_pose.pose.position.y += position_offset[1];
  target_pose.pose.position.z += position_offset[2];

  tf2::Quaternion current_orientation(
    current_pose.pose.orientation.x,
    current_pose.pose.orientation.y,
    current_pose.pose.orientation.z,
    current_pose.pose.orientation.w);
  current_orientation.normalize();

  tf2::Quaternion delta_orientation;
  delta_orientation.setRPY(rpy_offset[0], rpy_offset[1], rpy_offset[2]);
  delta_orientation.normalize();

  const tf2::Quaternion target_orientation = (current_orientation * delta_orientation).normalize();
  target_pose.pose.orientation.x = target_orientation.x();
  target_pose.pose.orientation.y = target_orientation.y();
  target_pose.pose.orientation.z = target_orientation.z();
  target_pose.pose.orientation.w = target_orientation.w();

  return target_pose;
}

bool ImpedanceCommandClient::command_relative_to_current(
  const CartesianImpedanceGains & gains,
  const std::array<double, 3> & position_offset,
  const std::array<double, 3> & rpy_offset,
  const std::string & current_pose_topic,
  double publish_seconds,
  double publish_rate_hz,
  std::chrono::nanoseconds timeout) const {
  const auto current_pose = wait_for_current_pose(current_pose_topic, timeout);
  if (!current_pose) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "No current pose received on '%s'. Is pose_broadcaster active?",
      current_pose_topic.c_str());
    return false;
  }

  if (!switch_to_controller(timeout)) {
    return false;
  }

  if (!set_gains(gains, timeout)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to set impedance gains.");
    return false;
  }

  auto target_pose = make_relative_target_pose(*current_pose, position_offset, rpy_offset);
  rclcpp::WallRate publish_rate(publish_rate_hz);
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(publish_seconds);

  do {
    target_pose.header.stamp = node_->now();
    publish_target_pose(target_pose);
    rclcpp::spin_some(node_);
    publish_rate.sleep();
  } while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline);

  return true;
}

bool ImpedanceCommandClient::command_target_pose(
  const CartesianImpedanceGains & gains,
  const std::array<double, 3> & target_position,
  const std::array<double, 4> & target_orientation_xyzw,
  double publish_seconds,
  double publish_rate_hz,
  std::chrono::nanoseconds timeout) const {
  if (!switch_to_controller(timeout)) {
    return false;
  }

  if (!set_gains(gains, timeout)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to set impedance gains.");
    return false;
  }

  auto target_pose = make_target_pose(target_position, target_orientation_xyzw);
  rclcpp::WallRate publish_rate(publish_rate_hz);
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(publish_seconds);

  do {
    target_pose.header.stamp = node_->now();
    publish_target_pose(target_pose);
    rclcpp::spin_some(node_);
    publish_rate.sleep();
  } while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline);

  return true;
}

bool ImpedanceCommandClient::command_target_pose(
  const CartesianImpedanceGains & gains,
  const PoseMatrix & target_pose_matrix,
  double publish_seconds,
  double publish_rate_hz,
  std::chrono::nanoseconds timeout) const {
  if (!switch_to_controller(timeout)) {
    return false;
  }

  if (!set_gains(gains, timeout)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to set impedance gains.");
    return false;
  }

  auto target_pose = make_target_pose(target_pose_matrix);
  rclcpp::WallRate publish_rate(publish_rate_hz);
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(publish_seconds);

  do {
    target_pose.header.stamp = node_->now();
    publish_target_pose(target_pose);
    rclcpp::spin_some(node_);
    publish_rate.sleep();
  } while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline);

  return true;
}

bool command_relative_impedance(
  rclcpp::Node::SharedPtr node,
  const CartesianImpedanceGains & gains,
  const std::array<double, 3> & position_offset,
  const std::array<double, 3> & rpy_offset,
  const std::string & controller_name,
  const std::string & current_pose_topic,
  const std::string & target_pose_topic,
  double publish_seconds,
  double publish_rate_hz,
  std::chrono::nanoseconds timeout) {
  ImpedanceCommandClient client(std::move(node), controller_name, target_pose_topic);
  return client.command_relative_to_current(
    gains,
    position_offset,
    rpy_offset,
    current_pose_topic,
    publish_seconds,
    publish_rate_hz,
    timeout);
}

bool command_impedance_target_pose(
  rclcpp::Node::SharedPtr node,
  const CartesianImpedanceGains & gains,
  const std::array<double, 3> & target_position,
  const std::array<double, 4> & target_orientation_xyzw,
  const std::string & controller_name,
  const std::string & target_pose_topic,
  double publish_seconds,
  double publish_rate_hz,
  std::chrono::nanoseconds timeout) {
  ImpedanceCommandClient client(std::move(node), controller_name, target_pose_topic);
  return client.command_target_pose(
    gains,
    target_position,
    target_orientation_xyzw,
    publish_seconds,
    publish_rate_hz,
    timeout);
}

bool command_impedance_target_matrix(
  rclcpp::Node::SharedPtr node,
  const CartesianImpedanceGains & gains,
  const PoseMatrix & target_pose_matrix,
  const std::string & controller_name,
  const std::string & target_pose_topic,
  double publish_seconds,
  double publish_rate_hz,
  std::chrono::nanoseconds timeout) {
  ImpedanceCommandClient client(std::move(node), controller_name, target_pose_topic);
  return client.command_target_pose(
    gains,
    target_pose_matrix,
    publish_seconds,
    publish_rate_hz,
    timeout);
}

bool command_impedance_target_matrix(
  rclcpp::Node::SharedPtr node,
  const std::array<double, 6> & stiffness,
  const std::array<double, 6> & damping,
  const PoseMatrix & target_pose_matrix,
  const std::string & controller_name,
  const std::string & target_pose_topic,
  double publish_seconds,
  double publish_rate_hz,
  std::chrono::nanoseconds timeout) {
  return command_impedance_target_matrix(
    std::move(node),
    make_cartesian_impedance_gains(stiffness, damping),
    target_pose_matrix,
    controller_name,
    target_pose_topic,
    publish_seconds,
    publish_rate_hz,
    timeout);
}

class ImpedanceController final : public crisp_controllers::CartesianController {};

}  // namespace my_controller

PLUGINLIB_EXPORT_CLASS(my_controller::ImpedanceController, controller_interface::ControllerInterface)
