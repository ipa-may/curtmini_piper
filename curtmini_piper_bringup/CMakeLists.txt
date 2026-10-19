cmake_minimum_required(VERSION 3.8)
project(curtmini_piper_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/home_arm.py
    scripts/joint_state_prefix_bridge.py
    scripts/robot_description_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_startup_homing test/test_startup_homing.py)
endif()

ament_package()
