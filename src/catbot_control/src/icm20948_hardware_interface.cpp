#include "catbot_control/icm20948_hardware_interface.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include "rclcpp/rclcpp.hpp"

namespace catbot_control {

ICM_20948_Status_e ICM20948HardwareInterface::i2c_write_cb(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user) {
  auto *hw = static_cast<ICM20948HardwareInterface *>(user);
  if (hw->i2c_fd_ < 0) return ICM_20948_Stat_Err;

  std::vector<uint8_t> buf;
  buf.reserve(len + 1);
  buf.push_back(regaddr);
  buf.insert(buf.end(), pdata, pdata + len);

  if (::write(hw->i2c_fd_, buf.data(), buf.size()) != static_cast<ssize_t>(buf.size())) {
    return ICM_20948_Stat_Err;
  }
  return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e ICM20948HardwareInterface::i2c_read_cb(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user) {
  auto *hw = static_cast<ICM20948HardwareInterface *>(user);
  if (hw->i2c_fd_ < 0) return ICM_20948_Stat_Err;

  if (::write(hw->i2c_fd_, &regaddr, 1) != 1) {
    return ICM_20948_Stat_Err;
  }
  if (::read(hw->i2c_fd_, pdata, len) != static_cast<ssize_t>(len)) {
    return ICM_20948_Stat_Err;
  }
  return ICM_20948_Stat_Ok;
}

hardware_interface::CallbackReturn ICM20948HardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params) {
  if (hardware_interface::SensorInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set up the vtable
  icm_serif_.write = i2c_write_cb;
  icm_serif_.read = i2c_read_cb;
  icm_serif_.user = this;

  hw_sensor_states_.resize(10, 0.0); // 3 accel, 3 gyro

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ICM20948HardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu", "orientation.x", &hw_sensor_states_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu", "orientation.y", &hw_sensor_states_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu", "orientation.z", &hw_sensor_states_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu", "orientation.w", &hw_sensor_states_[3]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu", "linear_acceleration.x", &hw_sensor_states_[4]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu", "linear_acceleration.y", &hw_sensor_states_[5]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu", "linear_acceleration.z", &hw_sensor_states_[6]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu", "angular_velocity.x", &hw_sensor_states_[7]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu", "angular_velocity.y", &hw_sensor_states_[8]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu", "angular_velocity.z", &hw_sensor_states_[9]));

  return state_interfaces;
}

hardware_interface::CallbackReturn ICM20948HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/) {
  i2c_fd_ = open(i2c_device_.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ICM20948HardwareInterface"), "Failed to open I2C device");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (ioctl(i2c_fd_, I2C_SLAVE, i2c_address_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ICM20948HardwareInterface"), "Failed to set I2C address");
    close(i2c_fd_);
    i2c_fd_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  ICM_20948_init_struct(&icm_device_);
  ICM_20948_link_serif(&icm_device_, &icm_serif_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ICM20948HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/) {
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
    i2c_fd_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ICM20948HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

  ICM_20948_AGMT_t agmt;
  // Read sensor data (assuming some data update function, simple raw read for example)
  ICM_20948_get_agmt(&icm_device_, &agmt);

  hw_sensor_states_[4] = agmt.acc.axes.x;
  hw_sensor_states_[5] = agmt.acc.axes.y;
  hw_sensor_states_[6] = agmt.acc.axes.z;
  hw_sensor_states_[7] = agmt.gyr.axes.x;
  hw_sensor_states_[8] = agmt.gyr.axes.y;
  hw_sensor_states_[9] = agmt.gyr.axes.z;

  return hardware_interface::return_type::OK;
}

}  // namespace catbot_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(catbot_control::ICM20948HardwareInterface,
                       hardware_interface::SensorInterface)
