cmake_minimum_required(VERSION 3.8)
project(sml_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/arm_commander.py
    scripts/arm_kinematics.py
    scripts/arm_control_gui.py
    scripts/pick_and_place_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
