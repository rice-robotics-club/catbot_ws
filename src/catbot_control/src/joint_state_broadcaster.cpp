#include "catbot_control/joint_state_broadcaster.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <controller_interface/controller_interface_base.hpp>

namespace catbot_control {

JointStateBroadcaster::JointStateBroadcaster() {}

controller_interface::CallbackReturn JointStateBroadcaster::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception &e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n",
            e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointStateBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
JointStateBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type =
      controller_interface::interface_configuration_type::ALL;
  return state_interfaces_config;
}



} // namespace catbot_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(catbot_control::JointStateBroadcaster,
                       controller_interface::ControllerInterface)
