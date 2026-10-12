// Gazebo Harmonic world (SDF) and ros_gz_bridge config generation --
// the C++ counterpart of build_mujoco_xml(num_obs) in the NMPC script.
// ROS-free: used by the make_world tool.
#pragma once

#include <string>
#include <vector>

#include "av_rrt_pid/types.hpp"

namespace av_rrt_pid
{

struct WorldSpec
{
  std::string world_name{"av_rrt_world"};
  std::vector<Obstacle> obstacles;   // only x, y are used for the initial poses
  double obs_radius{0.30};
  double safe_radius{0.90};
  double lane_y{2.0};
  Point2 goal{25.0, 2.0};
  double start_x{0.0};
  double start_y{0.0};
  double start_yaw{0.0};
};

// Same scene as the MuJoCo version: ground, green reference lane, blue car
// with cabin / hood / LiDAR block and two beams, red sphere obstacles with
// translucent safety discs. Every moving body gets Gazebo's VelocityControl
// and OdometryPublisher systems.
std::string buildWorldSdf(const WorldSpec & spec);

// ros_gz_bridge config (YAML) for /clock, the car and every obstacle.
std::string buildBridgeConfig(size_t num_obstacles);

}  // namespace av_rrt_pid
