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
find_package(std_msgs REQUIRED)

add_library(robot_hardware_backends SHARED
  src/chassis_packet.cpp
  src/chassis_kinematics.cpp
)

# 公共头文件路径
target_include_directories(robot_hardware_backends PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 向使用这个库的 target 传递 C++17 要求
target_compile_features(robot_hardware_backends PUBLIC cxx_std_17)


add_executable(test_kinematics src/test_kinematics.cpp)
target_link_libraries(test_kinematics robot_hardware_backends)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 安装库并导出 CMake target
install(
  TARGETS robot_hardware_backends
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 导出公共接口，供其他 ROS 2 package 使用
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
