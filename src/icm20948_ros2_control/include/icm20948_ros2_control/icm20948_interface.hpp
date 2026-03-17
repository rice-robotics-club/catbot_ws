#ifndef ICM20948_ROS2_CONTROL__ICM20948_INTERFACE_HPP_
#define ICM20948_ROS2_CONTROL__ICM20948_INTERFACE_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

#include "icm20948/ICM_20948_C.h"

namespace icm20948_ros2_control {
class ICM20948Interface : public hardware_interface::SensorInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ICM20948Interface)

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
  // I2C userspace fields

  /**
   * @brief File descriptor for the I2C device. Not initialized if -1.
   */
  int i2c_fd_ = -1;

  /**
   * @brief The I2C device file, defaults to /dev/i2c-1.
   */
  std::string i2c_device_ = "/dev/i2c-1";

  /**
   * @brief The I2C address of the device, defaults to 0x69.
   */
  int i2c_address_ = 0x69;

  // ICM20948 structs

  /**
   * @brief The ICM20948 device struct, used for reading and configuring IMU.
   */
  ICM_20948_Device_t icm_device_;

  /**
   * @brief The ICM20948 serial interface struct, used for providing I2C read/write callbacks.
   */
  ICM_20948_Serif_t icm_serif_;

  // HW sensor state fields

  /**
   * @brief The orientation of the sensor, as a quaternion.
   */
  std::array<double, 4> hw_sensor_orientation_;

  /**
   * @brief The angular velocity of the sensor, as a 3D vector.
   */
  std::array<double, 3> hw_sensor_angular_velocity_;

  /**
   * @brief The linear acceleration of the sensor, as a 3D vector.
   */
  std::array<double, 3> hw_sensor_linear_acceleration_;

  // static ICM20948 serial interface callbacks

  /**
   * @brief Static I2C write callback function, used by the ICM20948 serial interface struct.
   */
  static ICM_20948_Status_e i2c_write_cb(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);

  /**
   * @brief Static I2C read callback function, used by the ICM20948 serial interface struct.
   */
  static ICM_20948_Status_e i2c_read_cb(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
};
} // namespace icm20948_ros2_control

#endif // ICM20948_ROS2_CONTROL__ICM20948_HARDWARE_INTERFACE_HPP_
