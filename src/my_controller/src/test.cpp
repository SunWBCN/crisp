#include <my_controller/impedance_controller.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("impedance_controller_test");

  const std::array<double, 6> stiffness = {200.0, 200.0, 200.0, 30.0, 30.0, 30.0};
  const std::array<double, 6> damping = {60.0, 60.0, 60.0, 10.0, 10.0, 10.0};

  const my_controller::PoseMatrix target_pose_matrix = {{
    {{0.846582967, 0.249981819, 0.469900384, 0.703912390}},
    {{0.294614962, -0.955349966, -0.022549219, -0.015399744}},
    {{0.443282421, 0.157529469, -0.882431392, 0.182804495}},
    {{0.000000000, 0.000000000, 0.000000000, 1.000000000}},
  }};

  const bool ok = my_controller::command_impedance_target_matrix(
    node,
    stiffness,
    damping,
    target_pose_matrix);

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
