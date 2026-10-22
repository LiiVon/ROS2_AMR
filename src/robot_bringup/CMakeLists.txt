cmake_minimum_required(VERSION 3.20)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_mock_bringup.launch.py TIMEOUT 30)

  # 代码规范检查（需求文档 TR-10）：命名、禁用词、单文件行数
  add_test(NAME conventions
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/check_conventions.py
            --root ${CMAKE_CURRENT_SOURCE_DIR}/.. --strict)
endif()

ament_package()
