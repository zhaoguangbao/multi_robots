cmake_minimum_required(VERSION 2.8.3)
project(ur_teleop)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  control_msgs
  actionlib
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ur_teleop_position_keyboard
  include/ur_teleop_position_keyboard.h
  src/ur_teleop_position_keyboard.cpp
)
add_executable(ur_teleop_trajectory_keyboard
  include/ur_teleop_trajectory_keyboard.h
  src/ur_teleop_trajectory_keyboard.cpp
)
add_executable(test_param
  src/test_param.cpp
)

target_link_libraries(
  ur_teleop_position_keyboard
  ${catkin_LIBRARIES}
)
target_link_libraries(
  ur_teleop_trajectory_keyboard
  ${catkin_LIBRARIES}
)
target_link_libraries(
  test_param
  ${catkin_LIBRARIES}
)
