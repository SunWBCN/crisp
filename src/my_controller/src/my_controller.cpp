#include <my_controller/my_controller.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace my_controller {

CallbackReturn MyController::on_init() {
  try {
    params_listener_ = std::make_shared<my_controller::ParamListener>(get_node());
    params_ = params_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MyController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  params_ = params_listener_->get_params();
  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified.");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MyController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : params_.joints) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
MyController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : params_.joints) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

CallbackReturn MyController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  // Hold the position the joints are in at activation time.
  q_desired_.resize(params_.joints.size());
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    q_desired_[i] = state_interfaces_[2 * i].get_optional().value_or(0.0);  // position interface
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn MyController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  for (auto & command_interface : command_interfaces_) {
    (void)command_interface.set_value(0.0);
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type
MyController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (params_listener_->is_old(params_)) {
    params_ = params_listener_->get_params();
  }

  // Simple PD law that holds the captured position. With kp = kd = 0 this
  // commands zero effort, i.e. gravity-compensated free motion on the FR3.
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    const double q = state_interfaces_[2 * i].get_optional().value_or(q_desired_[i]);
    const double q_dot = state_interfaces_[2 * i + 1].get_optional().value_or(0.0);
    const double tau = params_.kp * (q_desired_[i] - q) - params_.kd * q_dot;
    (void)command_interfaces_[i].set_value(tau);
  }

  return controller_interface::return_type::OK;
}

}  // namespace my_controller

PLUGINLIB_EXPORT_CLASS(my_controller::MyController, controller_interface::ControllerInterface)
