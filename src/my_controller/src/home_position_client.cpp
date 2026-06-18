#include <my_controller/home_position.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace my_controller {

namespace {

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

bool ends_with(const std::string & value, const std::string & suffix) {
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
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

bool switch_to_controller(
  const rclcpp::Node::SharedPtr & node,
  const std::string & controller_name,
  std::chrono::nanoseconds timeout) {
  auto list_client =
    node->create_client<controller_manager_msgs::srv::ListControllers>(
      "controller_manager/list_controllers");
  auto switch_client =
    node->create_client<controller_manager_msgs::srv::SwitchController>(
      "controller_manager/switch_controller");

  if (!list_client->wait_for_service(timeout) || !switch_client->wait_for_service(timeout)) {
    RCLCPP_ERROR(node->get_logger(), "controller_manager services are not available.");
    return false;
  }

  auto list_future =
    list_client->async_send_request(
      std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
  if (!wait_for_future(node, list_future, timeout)) {
    RCLCPP_ERROR(node->get_logger(), "Timed out while listing controllers.");
    return false;
  }

  const auto controllers = list_future.get()->controller;
  const auto target_controller =
    std::find_if(controllers.begin(), controllers.end(), [&controller_name](const auto & c) {
      return c.name == controller_name;
    });
  if (target_controller == controllers.end()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Controller '%s' is not loaded. Start the Franka launch file first.",
      controller_name.c_str());
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
  request->activate_controllers = {controller_name};
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
  request->activate_asap = true;
  request->timeout.sec = 5;
  request->timeout.nanosec = 0;

  auto switch_future = switch_client->async_send_request(request);
  if (!wait_for_future(node, switch_future, timeout)) {
    RCLCPP_ERROR(node->get_logger(), "Timed out while switching controllers.");
    return false;
  }

  if (!switch_future.get()->ok) {
    RCLCPP_ERROR(
      node->get_logger(), "Failed to switch to controller '%s'.", controller_name.c_str());
    return false;
  }

  return true;
}

}  // namespace

std::vector<std::string> franka_joint_names() {
  return {
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
  };
}

std::vector<double> franka_home_joint_positions() {
  return {
    0.0,
    -M_PI / 4.0,
    0.0,
    -3.0 * M_PI / 4.0,
    0.0,
    M_PI / 2.0,
    M_PI / 4.0,
  };
}

bool command_home_position(
  rclcpp::Node::SharedPtr node,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & joint_positions,
  const std::string & controller_name,
  double move_duration_seconds,
  std::chrono::nanoseconds timeout) {
  if (joint_names.size() != joint_positions.size()) {
    RCLCPP_ERROR(node->get_logger(), "joint_names and joint_positions must have the same size.");
    return false;
  }

  if (!switch_to_controller(node, controller_name, timeout)) {
    return false;
  }

  auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
    node, controller_name + "/follow_joint_trajectory");
  if (!action_client->wait_for_action_server(timeout)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "FollowJointTrajectory action server for '%s' is not available.",
      controller_name.c_str());
    return false;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = joint_names;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = joint_positions;
  point.time_from_start.sec = static_cast<int32_t>(move_duration_seconds);
  point.time_from_start.nanosec =
    static_cast<uint32_t>((move_duration_seconds - point.time_from_start.sec) * 1e9);
  goal.trajectory.points.push_back(point);

  auto goal_future = action_client->async_send_goal(goal);
  if (!wait_for_future(node, goal_future, timeout)) {
    RCLCPP_ERROR(node->get_logger(), "Timed out while sending the home trajectory goal.");
    return false;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Home trajectory goal was rejected.");
    return false;
  }

  auto result_future = action_client->async_get_result(goal_handle);
  if (!wait_for_future(
      node,
      result_future,
      timeout + std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(move_duration_seconds)))) {
    RCLCPP_ERROR(node->get_logger(), "Timed out while waiting for home trajectory result.");
    return false;
  }

  const GoalHandleFollowJointTrajectory::WrappedResult result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
      result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL) {
    RCLCPP_ERROR(node->get_logger(), "Home trajectory did not finish successfully.");
    return false;
  }

  return true;
}

bool command_franka_home_position(
  rclcpp::Node::SharedPtr node,
  const std::string & controller_name,
  double move_duration_seconds,
  std::chrono::nanoseconds timeout) {
  return command_home_position(
    std::move(node),
    franka_joint_names(),
    franka_home_joint_positions(),
    controller_name,
    move_duration_seconds,
    timeout);
}

}  // namespace my_controller
