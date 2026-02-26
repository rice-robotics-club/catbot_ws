cmake_minimum_required(VERSION 3.8)
project(catbot_control)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(urdf REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(joint_state_broadcaster_parameters
  src/joint_state_broadcaster_parameters.yaml
)

add_library(joint_state_broadcaster SHARED
  src/joint_state_broadcaster.cpp
)
target_compile_features(joint_state_broadcaster PUBLIC cxx_std_17)
target_include_directories(joint_state_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/joint_state_broadcaster>
)
target_link_libraries(joint_state_broadcaster PUBLIC
                      joint_state_broadcaster_parameters
                      controller_interface::controller_interface
                      pluginlib::pluginlib
                      rclcpp::rclcpp
                      rclcpp_lifecycle::rclcpp_lifecycle
                      realtime_tools::realtime_tools
                      urdf::urdf
                      ${sensor_msgs_TARGETS}
                      ${control_msgs_TARGETS}
                      ${builtin_interfaces_TARGETS})
pluginlib_export_plugin_description_file(controller_interface catbot_control_plugins.xml)

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/joint_state_broadcaster
)
install(
  TARGETS
    joint_state_broadcaster
    joint_state_broadcaster_parameters
  EXPORT export_joint_state_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_joint_state_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
