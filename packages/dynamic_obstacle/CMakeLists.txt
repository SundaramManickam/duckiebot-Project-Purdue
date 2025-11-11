cmake_minimum_required(VERSION 3.0.2)
project(dynamic_obstacle)  # must match package.xml <name>

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
  src/tof_reader.py
  src/ipm_lane_occupancy.py
  src/controller_pp.py
  src/fsm_avoid.py
  src/wheel_encoder_reader_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
