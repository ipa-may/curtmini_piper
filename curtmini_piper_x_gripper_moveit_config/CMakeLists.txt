cmake_minimum_required(VERSION 3.8)
project(curtmini_piper_x_gripper_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
