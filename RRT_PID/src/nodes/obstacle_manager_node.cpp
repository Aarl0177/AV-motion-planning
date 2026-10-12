// obstacle_manager
//
// * Drives every obstacle in Gazebo at its constant velocity by publishing
//   /model/obs_j/cmd_vel (Gazebo's VelocityControl plugin) -- the Gazebo
//   version of obstacle_dynamics_all().
// * Reads their positions back from /model/obs_j/odometry and publishes the
//   whole set on /obstacles as [x, y, vx, vy] per obstacle (the obs_states
//   array of the Python script) for the planner.
// * Publishes RViz markers (obstacle + safety disc).
//
// The obstacles stay still until the planner publishes its first path, so
// the car and the obstacles start moving at the same moment, like t = 0 in
// the Python simulation.

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class ObstacleManager : public rclcpp::Node
{
public:
  ObstacleManager()
  : Node("obstacle_manager")
  {
    const auto flat = declare_parameter<std::vector<double>>("obstacles", std::vector<double>{});
    obs_radius_ = declare_parameter<double>("obs_radius", 0.30);
    const double vehicle_radius = declare_parameter<double>("vehicle_radius", 0.20);
    const double margin = declare_parameter<double>("margin", 0.40);
    safe_radius_ = obs_radius_ + vehicle_radius + margin;
    wait_for_path_ = declare_parameter<bool>("wait_for_path", true);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");

    if (flat.empty() || flat.size() % 4 != 0) {
      throw std::runtime_error("parameter 'obstacles' must be a flat list [x, y, vx, vy, ...]");
    }
    const size_t n = flat.size() / 4;
    state_.assign(flat.begin(), flat.end());  // positions are overwritten by odometry
    cmd_.resize(n);
    for (size_t j = 0; j < n; ++j) {
      cmd_[j] = {flat[4 * j + 2], flat[4 * j + 3]};
    }

    for (size_t j = 0; j < n; ++j) {
      const std::string base = "/model/obs_" + std::to_string(j);
      cmd_pubs_.push_back(create_publisher<geometry_msgs::msg::Twist>(base + "/cmd_vel", 10));
      odom_subs_.push_back(
        create_subscription<nav_msgs::msg::Odometry>(
          base + "/odometry", 10,
          [this, j](const nav_msgs::msg::Odometry::SharedPtr msg) {
            state_[4 * j + 0] = msg->pose.pose.position.x;
            state_[4 * j + 1] = msg->pose.pose.position.y;
          }));
    }

    obstacles_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/obstacles", 10);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_markers", 10);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {
        if (!moving_ && !msg->poses.empty()) {
          moving_ = true;
          RCLCPP_INFO(get_logger(), "First path received -> obstacles start moving");
        }
      });
    moving_ = !wait_for_path_;

    timer_ = create_wall_timer(50ms, [this]() {publish();});
    RCLCPP_INFO(get_logger(), "Managing %zu obstacles (safe radius %.2f m)", n, safe_radius_);
  }

private:
  void publish()
  {
    const size_t n = cmd_.size();

    // 1) velocity commands to Gazebo (body frame == world frame, obstacles don't rotate)
    for (size_t j = 0; j < n; ++j) {
      geometry_msgs::msg::Twist tw;
      if (moving_) {
        tw.linear.x = cmd_[j].first;
        tw.linear.y = cmd_[j].second;
      }
      cmd_pubs_[j]->publish(tw);
    }

    // 2) obstacle states for the planner. The velocity is the commanded one,
    //    i.e. the planner is told the true constant velocity, exactly like the
    //    NMPC script passed obs_states to predict_obstacle(). With real sensors
    //    this would come from a tracker (e.g. a Kalman filter) instead.
    std_msgs::msg::Float64MultiArray arr;
    arr.data.resize(4 * n);
    for (size_t j = 0; j < n; ++j) {
      arr.data[4 * j + 0] = state_[4 * j + 0];
      arr.data[4 * j + 1] = state_[4 * j + 1];
      arr.data[4 * j + 2] = cmd_[j].first;
      arr.data[4 * j + 3] = cmd_[j].second;
    }
    obstacles_pub_->publish(arr);

    // 3) RViz markers
    visualization_msgs::msg::MarkerArray ma;
    const auto stamp = now();
    for (size_t j = 0; j < n; ++j) {
      visualization_msgs::msg::Marker body;
      body.header.frame_id = frame_id_;
      body.header.stamp = stamp;
      body.ns = "obstacles";
      body.id = static_cast<int>(j);
      body.type = visualization_msgs::msg::Marker::SPHERE;
      body.action = visualization_msgs::msg::Marker::ADD;
      body.pose.position.x = state_[4 * j];
      body.pose.position.y = state_[4 * j + 1];
      body.pose.position.z = 0.3;
      body.pose.orientation.w = 1.0;
      body.scale.x = body.scale.y = body.scale.z = 2.0 * obs_radius_;
      body.color.r = 0.9f;
      body.color.g = 0.1f;
      body.color.b = 0.1f;
      body.color.a = 1.0f;
      ma.markers.push_back(body);

      visualization_msgs::msg::Marker zone = body;
      zone.ns = "safe_zone";
      zone.type = visualization_msgs::msg::Marker::CYLINDER;
      zone.pose.position.z = 0.01;
      zone.scale.x = zone.scale.y = 2.0 * safe_radius_;
      zone.scale.z = 0.02;
      zone.color.a = 0.2f;
      ma.markers.push_back(zone);
    }
    markers_pub_->publish(ma);
  }

  double obs_radius_{0.3};
  double safe_radius_{0.9};
  bool wait_for_path_{true};
  bool moving_{false};
  std::string frame_id_;

  std::vector<double> state_;                      // [x, y, vx, vy] * n
  std::vector<std::pair<double, double>> cmd_;     // commanded (vx, vy)

  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_pubs_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr obstacles_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleManager>());
  rclcpp::shutdown();
  return 0;
}
