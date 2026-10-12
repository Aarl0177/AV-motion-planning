// Small helpers shared by the ROS 2 nodes (header-only, ROS-dependent).
#pragma once

#include <cmath>
#include <vector>

#include "av_rrt_pid/types.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace av_rrt_pid
{

inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

inline geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(0.5 * yaw);
  q.z = std::sin(0.5 * yaw);
  return q;
}

// Gazebo's OdometryPublisher gives the world pose and the body-frame twist,
// so twist.linear.x is the forward speed v of the bicycle model.
inline CarState carStateFromOdometry(const nav_msgs::msg::Odometry & msg)
{
  CarState s;
  s.x = msg.pose.pose.position.x;
  s.y = msg.pose.pose.position.y;
  s.psi = yawFromQuaternion(msg.pose.pose.orientation);
  s.v = msg.twist.twist.linear.x;
  return s;
}

// /obstacles is a Float64MultiArray laid out as [x, y, vx, vy] per obstacle,
// i.e. exactly the obs_states array of the Python script, flattened.
inline std::vector<Obstacle> obstaclesFromArray(const std_msgs::msg::Float64MultiArray & msg)
{
  std::vector<Obstacle> obs;
  for (size_t i = 0; i + 3 < msg.data.size(); i += 4) {
    obs.push_back({msg.data[i], msg.data[i + 1], msg.data[i + 2], msg.data[i + 3]});
  }
  return obs;
}

inline std::vector<Point2> pointsFromPath(const nav_msgs::msg::Path & msg)
{
  std::vector<Point2> pts;
  pts.reserve(msg.poses.size());
  for (const auto & p : msg.poses) {
    pts.push_back({p.pose.position.x, p.pose.position.y});
  }
  return pts;
}

}  // namespace av_rrt_pid
