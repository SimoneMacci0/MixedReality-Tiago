cmake_minimum_required(VERSION 3.0.2)
project(tiago_holo_dt)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_msgs
  ros_tcp_endpoint
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
)

## Generate services in the 'srv' folder
add_service_files(
   DIRECTORY srv
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   moveit_msgs
   sensor_msgs
   std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/motion_planner.py
  scripts/tiago_experiment.py
  scripts/base_footprint_pose_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

