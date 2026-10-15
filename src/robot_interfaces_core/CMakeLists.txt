cmake_minimum_required(VERSION 3.8)
project(robot_interfaces_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 把我们写的 .msg 交给 rosidl 生成 C++ 代码
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/TaskStatus.msg"
  "msg/ChassisState.msg"
  "msg/NavigationStatistics.msg"
  "msg/TrackingError.msg"
  "srv/SetControlMode.srv"
  "srv/SetChassisMode.srv"
  "srv/GetStatistics.srv"
  DEPENDENCIES std_msgs
)
ament_package()
