cmake_minimum_required(VERSION 3.8)
project(openrst_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(mc_daq_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/OpenrstRequest.srv"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Typesupport target for generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Libraries
add_library(${PROJECT_NAME}_lib SHARED
  src/openrst_control.cpp
  src/openrst_system.cpp
)
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}" Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  trajectory_msgs
  hardware_interface
  controller_manager
  pluginlib
  mc_daq_ros
)

# Pluginlib
pluginlib_export_plugin_description_file(hardware_interface openrst_control.xml)

# Executables
add_executable(openrst_control_node
  src/main.cpp
)
target_link_libraries(openrst_control_node ${PROJECT_NAME}_lib "${cpp_typesupport_target}")
ament_target_dependencies(openrst_control_node rclcpp)

# Install
install(TARGETS
  ${PROJECT_NAME}_lib
  openrst_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_package()