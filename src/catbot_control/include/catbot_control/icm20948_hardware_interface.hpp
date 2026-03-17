#ifndef CATBOT_CONTROL__ICM20948_HARDWARE_INTERFACE_HPP_
#define CATBOT_CONTROL__ICM20948_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

#include "icm20948/ICM_20948_C.h"

namespace catbot_control {
class ICM20948HardwareInterface : public hardware_interface::SensorInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ICM20948HardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  int i2c_fd_ = -1;
  std::string i2c_device_ = "/dev/i2c-1";
  int i2c_address_ = 0x69;

  ICM_20948_Device_t icm_device_;
  ICM_20948_Serif_t icm_serif_;

  std::vector<double> hw_sensor_states_;

  static ICM_20948_Status_e i2c_write_cb(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
  static ICM_20948_Status_e i2c_read_cb(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
};
} // namespace catbot_control

#endif // CATBOT_CONTROL__ICM20948_HARDWARE_INTERFACE_HPP_
