#include <my_controller/home_position.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("franka_home_position");

  const bool ok = my_controller::command_franka_home_position(node);

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
