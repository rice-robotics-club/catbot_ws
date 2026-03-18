cmake_minimum_required(VERSION 3.8)
project(catbot_control)

# find dependencies
find_package(ros2_control_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(urdf REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(generate_parameter_library REQUIRED)

# generate_parameter_library(joint_state_broadcaster_parameters
#   src/joint_state_broadcaster_parameters.yaml
# )

# add_library(${PROJECT_NAME} SHARED
#   # src/joint_state_broadcaster.cpp\
# )
# target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
#   $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/external>
#   $<INSTALL_INTERFACE:include>
# )
# target_link_libraries(${PROJECT_NAME} PUBLIC
#                       # joint_state_broadcaster_parameters
#                       controller_interface::controller_interface
#                       hardware_interface::hardware_interface
#                       pluginlib::pluginlib
#                       rclcpp::rclcpp
#                       rclcpp_lifecycle::rclcpp_lifecycle
#                       realtime_tools::realtime_tools
#                       urdf::urdf
#                       ${sensor_msgs_TARGETS}
#                       ${control_msgs_TARGETS}
#                       ${builtin_interfaces_TARGETS})

# Export Plugins
# pluginlib_export_plugin_description_file(controller_interface catbot_control_plugins.xml)

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  urdf
)
ament_package()
