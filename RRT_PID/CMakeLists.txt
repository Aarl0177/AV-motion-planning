cmake_minimum_required(VERSION 3.8)
project(av_rrt_pid)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ---------------------------------------------------------------------------
# ROS-free core: RRT planner, PID path follower, bicycle model.
# The nodes and the standalone simulator both link this, so what you tune
# offline is exactly what runs in Gazebo.
# ---------------------------------------------------------------------------
add_library(av_core STATIC
  src/core/rrt_planner.cpp
  src/core/local_planner.cpp
  src/core/pid_follower.cpp
  src/core/world_sdf.cpp
)
target_include_directories(av_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(av_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Offline tools (no ROS needed to run them)
add_executable(standalone_sim src/standalone_sim.cpp)
target_link_libraries(standalone_sim av_core)
add_executable(plot_results src/tools/plot_results.cpp)

# World + bridge-config generator (reads params.yaml through ROS parameters)
add_executable(make_world src/tools/make_world.cpp)
target_link_libraries(make_world av_core)
ament_target_dependencies(make_world rclcpp)

# ROS 2 nodes
foreach(node obstacle_manager_node rrt_planner_node pid_controller_node)
  add_executable(${node} src/nodes/${node}.cpp)
  target_link_libraries(${node} av_core)
  ament_target_dependencies(${node} rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs)
endforeach()

install(TARGETS
  standalone_sim
  plot_results
  make_world
  obstacle_manager_node
  rrt_planner_node
  pid_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config rviz worlds DESTINATION share/${PROJECT_NAME})

ament_package()
