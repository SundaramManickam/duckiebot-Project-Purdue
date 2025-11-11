cmake_minimum_required(VERSION 3.0.2)
project(duckie_race_avoid)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  duckietown_msgs
  cv_bridge
  image_transport
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS
  nodes/tof_reader.py
  nodes/ipm_lane_occupancy.py
  nodes/controller_pp.py
  nodes/fsm_avoid.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
