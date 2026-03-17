#ifndef CATBOT_CONTROL__JOINT_STATE_BROADCASTER_HPP_
#define CATBOT_CONTROL__JOINT_STATE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "rclcpp/version.h"
#if RCLCPP_VERSION_GTE(29, 0, 0)
#include "urdf/model.hpp"
#else
#include "urdf/model.h"
#endif

#include "catbot_control/joint_state_broadcaster_parameters.hpp"

namespace catbot_control {
class JointStateBroadcaster : public controller_interface::ControllerInterface {
public:
  JointStateBroadcaster();

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  controller_interface::CallbackReturn on_init() override;

  // controller_interface::CallbackReturn
  // on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

private:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  std::string frame_id_;
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, double> joint_positions_;
  std::unordered_map<std::string, double> joint_velocities_;
  std::unordered_map<std::string, double> joint_efforts_;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>>
      joint_state_publisher_;
  std::shared_ptr<
      realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>
      realtime_joint_state_publisher_;
  sensor_msgs::msg::JointState joint_state_msg_;

  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::DynamicJointState>>
      dynamic_joint_state_publisher_;
  std::shared_ptr<
      realtime_tools::RealtimePublisher<control_msgs::msg::DynamicJointState>>
      realtime_dynamic_joint_state_publisher_;
  control_msgs::msg::DynamicJointState dynamic_joint_state_msg_;
};
} // namespace catbot_control

#endif // CATBOT_CONTROL__JOINT_STATE_BROADCASTER_HPP_
