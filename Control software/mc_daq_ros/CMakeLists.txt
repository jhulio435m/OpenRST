cmake_minimum_required(VERSION 3.8)
project(mc_daq_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# Local uldaq installation path
set(ULDAQ_LOCAL_PATH "/home/jhulio/uldaq_install")

# Find uldaq
find_library(ULDAQ_LIB uldaq HINTS ${ULDAQ_LOCAL_PATH}/lib)
find_path(ULDAQ_INCLUDE_DIR uldaq.h HINTS ${ULDAQ_LOCAL_PATH}/include)

if(NOT ULDAQ_LIB OR NOT ULDAQ_INCLUDE_DIR)
  # Fallback to system-wide search
  find_library(ULDAQ_LIB uldaq)
  find_path(ULDAQ_INCLUDE_DIR uldaq.h)
endif()

if(NOT ULDAQ_LIB)
  message(FATAL_ERROR "uldaq library not found. Please install libuldaq (https://github.com/mccdaq/uldaq)")
endif()

# Generate Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DaqCmd.srv"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(
  include
  ${ULDAQ_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
)

# Typesupport target for generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Libraries
add_library(usb1608 src/usb1608/usb1608.cpp)
target_link_libraries(usb1608 ${ULDAQ_LIB} "${cpp_typesupport_target}")
ament_target_dependencies(usb1608 rclcpp std_msgs)

add_library(usb3104 src/usb3104/usb3104.cpp)
target_link_libraries(usb3104 ${ULDAQ_LIB} "${cpp_typesupport_target}")
ament_target_dependencies(usb3104 rclcpp std_msgs)

add_library(usbquad08 src/usbquad08/usbquad08.cpp)
target_link_libraries(usbquad08 ${ULDAQ_LIB} "${cpp_typesupport_target}")
ament_target_dependencies(usbquad08 rclcpp std_msgs)

# Executables
add_executable(mc_daq_usb1608_stream src/usb1608_stream.cpp)
target_link_libraries(mc_daq_usb1608_stream usb1608 "${cpp_typesupport_target}")
ament_target_dependencies(mc_daq_usb1608_stream rclcpp std_msgs)

add_executable(mc_daq_usb3104_stream src/usb3104_stream.cpp)
target_link_libraries(mc_daq_usb3104_stream usb3104 "${cpp_typesupport_target}")
ament_target_dependencies(mc_daq_usb3104_stream rclcpp std_msgs)

add_executable(mc_daq_usbquad08_stream src/usbquad08_stream.cpp)
target_link_libraries(mc_daq_usbquad08_stream usbquad08 "${cpp_typesupport_target}")
ament_target_dependencies(mc_daq_usbquad08_stream rclcpp std_msgs)

# Install libraries
install(TARGETS usb1608 usb3104 usbquad08
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  mc_daq_usb1608_stream
  mc_daq_usb3104_stream
  mc_daq_usbquad08_stream
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include ${ULDAQ_INCLUDE_DIR})
ament_export_libraries(usb1608 usb3104 usbquad08)
ament_package()