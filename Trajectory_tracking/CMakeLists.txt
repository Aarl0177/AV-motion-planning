cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  ackermann_msgs
  visualization_msgs
  geometry_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/pure_pursuit.py
  scripts/waypoint_logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
