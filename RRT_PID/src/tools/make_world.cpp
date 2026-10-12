/*
 * make_world -- writes the Gazebo world and the ros_gz_bridge config from
 * params.yaml, so the obstacles are defined in one place.
 *
 *   ros2 run av_rrt_pid make_world --ros-args \
 *       --params-file src/av_rrt_pid/config/params.yaml -p output_dir:=$HOME/av_sim
 *
 * then launch with the generated files:
 *
 *   ros2 launch av_rrt_pid sim.launch.xml \
 *       world:=$HOME/av_sim/av_rrt_world.sdf bridge_config:=$HOME/av_sim/bridge.yaml
 *
 * The package already ships both files for the default params.yaml, so you
 * only need this after changing obstacles, the lane, the goal or the start.
 */

#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "av_rrt_pid/world_sdf.hpp"
#include "rclcpp/rclcpp.hpp"

namespace av = av_rrt_pid;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("make_world");

  av::WorldSpec spec;
  const auto flat = node->declare_parameter<std::vector<double>>("obstacles", std::vector<double>{});
  spec.obs_radius = node->declare_parameter<double>("obs_radius", 0.30);
  const double vehicle_radius = node->declare_parameter<double>("vehicle_radius", 0.20);
  const double margin = node->declare_parameter<double>("margin", 0.40);
  spec.safe_radius = spec.obs_radius + vehicle_radius + margin;
  spec.lane_y = node->declare_parameter<double>("lane_y", 2.0);
  const auto goal = node->declare_parameter<std::vector<double>>("goal", {25.0, 2.0});
  const auto start = node->declare_parameter<std::vector<double>>("start_pose", {0.0, 0.0, 0.0});
  spec.world_name = node->declare_parameter<std::string>("world_name", "av_rrt_world");
  const auto out_dir = node->declare_parameter<std::string>("output_dir", ".");

  if (flat.empty() || flat.size() % 4 != 0) {
    RCLCPP_ERROR(
      node->get_logger(),
      "'obstacles' must be a flat list [x, y, vx, vy, ...]. Did you pass --params-file?");
    rclcpp::shutdown();
    return 1;
  }
  for (size_t i = 0; i + 3 < flat.size(); i += 4) {
    spec.obstacles.push_back({flat[i], flat[i + 1], flat[i + 2], flat[i + 3]});
  }
  if (goal.size() >= 2) {
    spec.goal = {goal[0], goal[1]};
  }
  if (start.size() >= 3) {
    spec.start_x = start[0];
    spec.start_y = start[1];
    spec.start_yaw = start[2];
  }

  std::filesystem::create_directories(out_dir);
  const std::string world_path = out_dir + "/av_rrt_world.sdf";
  const std::string bridge_path = out_dir + "/bridge.yaml";
  std::ofstream(world_path) << av::buildWorldSdf(spec);
  std::ofstream(bridge_path) << av::buildBridgeConfig(spec.obstacles.size());

  RCLCPP_INFO(
    node->get_logger(),
    "Wrote %s and %s (%zu obstacles, safe radius %.2f m).\nLaunch with:\n"
    "  ros2 launch av_rrt_pid sim.launch.xml world:=%s bridge_config:=%s",
    world_path.c_str(), bridge_path.c_str(), spec.obstacles.size(), spec.safe_radius,
    world_path.c_str(), bridge_path.c_str());

  rclcpp::shutdown();
  return 0;
}
