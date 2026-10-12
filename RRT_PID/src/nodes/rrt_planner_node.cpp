// rrt_planner
//
// Subscribes : /model/car/odometry  (nav_msgs/Odometry, from Gazebo)
//              /obstacles           (std_msgs/Float64MultiArray, [x, y, vx, vy] per obstacle)
// Publishes  : /planned_path        (nav_msgs/Path)  -- followed by pid_controller
//              /planner_markers     (visualization_msgs/MarkerArray) -- RRT tree, goals
//
// Runs the LocalPlanner (time-aware RRT) every `plan_period` seconds of
// *simulation* time, triggered by odometry messages, so it stays in step
// with Gazebo even when the simulation runs slower than real time.

#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "av_rrt_pid/local_planner.hpp"
#include "av_rrt_pid/ros_conversions.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace av = av_rrt_pid;

class RRTPlannerNode : public rclcpp::Node
{
public:
  RRTPlannerNode()
  : Node("rrt_planner")
  {
    // ---- vehicle / scenario (shared with the other nodes) ----
    const double L = declare_parameter<double>("wheelbase", 1.0);
    const double steer_max = declare_parameter<double>("steer_max", M_PI / 10.0);
    const auto goal = declare_parameter<std::vector<double>>("goal", {25.0, 2.0});

    av::RRTParams rp;
    rp.obs_radius = declare_parameter<double>("obs_radius", rp.obs_radius);
    rp.vehicle_radius = declare_parameter<double>("vehicle_radius", rp.vehicle_radius);
    rp.margin = declare_parameter<double>("margin", rp.margin);
    rp.plan_speed = declare_parameter<double>("cruise_speed", rp.plan_speed);
    rp.min_turn_radius = L / std::tan(steer_max);

    // ---- RRT ----
    rp.step_size = declare_parameter<double>("rrt.step_size", rp.step_size);
    rp.goal_bias = declare_parameter<double>("rrt.goal_bias", rp.goal_bias);
    rp.connect_radius = declare_parameter<double>("rrt.connect_radius", rp.connect_radius);
    rp.max_iterations = declare_parameter<int>("rrt.max_iterations", rp.max_iterations);
    rp.plan_accel = declare_parameter<double>("rrt.plan_accel", rp.plan_accel);
    rp.prediction_horizon =
      declare_parameter<double>("rrt.prediction_horizon", rp.prediction_horizon);
    rp.check_resolution = declare_parameter<double>("rrt.check_resolution", rp.check_resolution);
    rp.smoothing_iterations =
      declare_parameter<int>("rrt.smoothing_iterations", rp.smoothing_iterations);
    rp.seed = static_cast<unsigned int>(declare_parameter<int>("rrt.seed", 0));
    const auto bounds = declare_parameter<std::vector<double>>(
      "rrt.bounds", {rp.x_min, rp.x_max, rp.y_min, rp.y_max});
    if (bounds.size() == 4) {
      rp.x_min = bounds[0];
      rp.x_max = bounds[1];
      rp.y_min = bounds[2];
      rp.y_max = bounds[3];
    }

    // ---- receding-horizon logic ----
    av::LocalPlannerParams lp;
    lp.lane_y = declare_parameter<double>("lane_y", lp.lane_y);
    if (goal.size() >= 2) {
      lp.final_goal = {goal[0], goal[1]};
    }
    lp.goal_lookahead = declare_parameter<double>("goal_lookahead", lp.goal_lookahead);
    lp.goal_reached_tol = declare_parameter<double>("goal_tolerance", lp.goal_reached_tol);
    lp.max_deviation = declare_parameter<double>("max_deviation", lp.max_deviation);
    lp.replan_remaining = declare_parameter<double>("replan_remaining", lp.replan_remaining);
    lp.plan_inflation = declare_parameter<double>("plan_inflation", lp.plan_inflation);

    plan_period_ = declare_parameter<double>("plan_period", 0.2);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/model/car/odometry");
    const auto log_prefix = declare_parameter<std::string>("log_prefix", "");
    if (!log_prefix.empty()) {
      path_log_.open(log_prefix + "_paths.csv");
      path_log_ << "plan_id,t,reason,x,y\n";
    }

    planner_ = std::make_unique<av::LocalPlanner>(lp, rp);

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/planner_markers", 10);
    obs_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/obstacles", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        obstacles_ = av::obstaclesFromArray(*msg);
        have_obstacles_ = true;
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {onOdometry(*msg);});

    RCLCPP_INFO(
      get_logger(), "RRT planner ready: goal (%.1f, %.1f), R_min %.2f m, safe radius %.2f m",
      lp.final_goal.x, lp.final_goal.y, rp.min_turn_radius, rp.safeRadius());
  }

private:
  void onOdometry(const nav_msgs::msg::Odometry & msg)
  {
    const double t = rclcpp::Time(msg.header.stamp).seconds();
    if (t < last_plan_t_) {
      last_plan_t_ = -1e9;  // simulation was reset
    }
    if (t - last_plan_t_ < plan_period_ - 1e-6) {
      return;
    }
    if (!have_obstacles_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for /obstacles ...");
      return;
    }
    last_plan_t_ = t;
    if (t0_ < 0.0) {
      t0_ = t;
    }

    const av::CarState car = av::carStateFromOdometry(msg);
    const auto status = planner_->update(car, obstacles_);

    switch (status) {
      case av::LocalPlanner::Status::kReplanned:
        RCLCPP_INFO(
          get_logger(), "t=%.1f replanned (%s): %zu waypoints, %zu tree nodes",
          t - t0_, planner_->lastReason().c_str(), planner_->path().size(),
          planner_->rrt().tree().size());
        logPath(t - t0_);
        publishMarkers(msg.header.stamp);
        break;
      case av::LocalPlanner::Status::kFailed:
        RCLCPP_WARN(get_logger(), "t=%.1f %s", t - t0_, planner_->lastReason().c_str());
        break;
      case av::LocalPlanner::Status::kGoalReached:
        if (!goal_reported_) {
          RCLCPP_INFO(get_logger(), "t=%.1f goal reached", t - t0_);
          goal_reported_ = true;
        }
        break;
      case av::LocalPlanner::Status::kKeep:
        break;
    }
    publishPath(msg.header.stamp);
  }

  void publishPath(const builtin_interfaces::msg::Time & stamp)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp = stamp;
    const auto & pts = planner_->path();
    for (size_t i = 0; i < pts.size(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = pts[i].x;
      ps.pose.position.y = pts[i].y;
      const size_t k = (i + 1 < pts.size()) ? i : (i > 0 ? i - 1 : i);
      const double yaw = (pts.size() > 1) ?
        std::atan2(pts[k + 1].y - pts[k].y, pts[k + 1].x - pts[k].x) : 0.0;
      ps.pose.orientation = av::quaternionFromYaw(yaw);
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);
  }

  void publishMarkers(const builtin_interfaces::msg::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker tree;
    tree.header.frame_id = frame_id_;
    tree.header.stamp = stamp;
    tree.ns = "rrt_tree";
    tree.id = 0;
    tree.type = visualization_msgs::msg::Marker::LINE_LIST;
    tree.action = visualization_msgs::msg::Marker::ADD;
    tree.pose.orientation.w = 1.0;
    tree.scale.x = 0.015;
    tree.color.r = 0.6f;
    tree.color.g = 0.6f;
    tree.color.b = 0.6f;
    tree.color.a = 0.5f;
    for (const auto & n : planner_->rrt().tree()) {
      if (n.parent < 0) {
        continue;
      }
      const auto & p = planner_->rrt().tree()[n.parent].p;
      geometry_msgs::msg::Point a, b;
      a.x = p.x;
      a.y = p.y;
      a.z = 0.02;
      b.x = n.p.x;
      b.y = n.p.y;
      b.z = 0.02;
      tree.points.push_back(a);
      tree.points.push_back(b);
    }
    ma.markers.push_back(tree);

    auto disc = [&](const std::string & ns, const av::Point2 & p, float r, float g, float b) {
        visualization_msgs::msg::Marker m;
        m.header = tree.header;
        m.ns = ns;
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = p.x;
        m.pose.position.y = p.y;
        m.pose.position.z = 0.02;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = 0.4;
        m.scale.z = 0.04;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 0.9f;
        return m;
      };
    ma.markers.push_back(disc("local_goal", planner_->localGoal(), 1.0f, 0.6f, 0.0f));
    ma.markers.push_back(disc("final_goal", planner_->params().final_goal, 0.1f, 0.8f, 0.1f));
    markers_pub_->publish(ma);
  }

  void logPath(double t)
  {
    if (!path_log_.is_open()) {
      return;
    }
    for (const auto & p : planner_->path()) {
      path_log_ << plan_id_ << "," << t << "," << planner_->lastReason() << ","
                << p.x << "," << p.y << "\n";
    }
    path_log_.flush();
    ++plan_id_;
  }

  std::unique_ptr<av::LocalPlanner> planner_;
  std::vector<av::Obstacle> obstacles_;
  bool have_obstacles_{false};
  bool goal_reported_{false};
  double plan_period_{0.2};
  double last_plan_t_{-1e9};
  double t0_{-1.0};
  int plan_id_{0};
  std::string frame_id_;
  std::ofstream path_log_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obs_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RRTPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
