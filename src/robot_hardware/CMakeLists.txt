cmake_minimum_required(VERSION 3.8)
project(robot_hardware)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(robot_interfaces_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(robot_hardware_backends SHARED
  src/chassis_packet.cpp
  src/chassis_kinematics.cpp
  src/chassis_backend_factory.cpp
  src/mock_backend.cpp
  src/serial_backend.cpp
  src/udp_backend.cpp
  src/chassis_system_adapter.cpp
)
target_include_directories(robot_hardware_backends PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(robot_hardware_backends PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(robot_chassis_hardware SHARED
  src/chassis_hardware_interface.cpp)
target_link_libraries(robot_chassis_hardware robot_hardware_backends)
ament_target_dependencies(robot_chassis_hardware
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface robot_hardware_plugins.xml)

install(
  TARGETS robot_hardware_backends robot_chassis_hardware
  EXPORT export_robot_hardware_backends
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES robot_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_robot_hardware_backends HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib)

ament_package()
