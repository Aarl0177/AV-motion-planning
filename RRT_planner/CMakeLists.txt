cmake_minimum_required(VERSION 3.0.2)
project(rrt_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  ackermann_msgs
  visualization_msgs
  geometry_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/rrt_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)