// pid_controller
//
// Subscribes : /model/car/odometry  (nav_msgs/Odometry, from Gazebo)
//              /planned_path        (nav_msgs/Path, from rrt_planner)
//              /obstacles           (only for the CSV log)
// Publishes  : /model/car/cmd_vel   (geometry_msgs/Twist, to Gazebo's VelocityControl)
//              /car/control         (std_msgs/Float64MultiArray [a, delta, v_cmd, v_target])
//
// The PathFollower produces the same inputs the NMPC produced, u = [a, delta].
// They are turned into a Gazebo velocity command with the kinematic bicycle
// model:   v_cmd <- v_cmd + a * dt,     omega = v_cmd * tan(delta) / L
// so the car in Gazebo moves exactly like AV.dynamics() in the Python code.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "av_rrt_pid/pid_follower.hpp"
#include "av_rrt_pid/ros_conversions.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace av = av_rrt_pid;

class PIDControllerNode : public rclcpp::Node
{
public:
  PIDControllerNode()
  : Node("pid_controller"), follower_(loadParams())
  {
    control_period_ = declare_parameter<double>("control_period", 0.1);
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/model/car/odometry");
    const auto cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "/model/car/cmd_vel");
    const auto log_prefix = declare_parameter<std::string>("log_prefix", "");
    if (!log_prefix.empty()) {
      log_.open(log_prefix + "_log.csv");
      RCLCPP_INFO(get_logger(), "Logging to %s_log.csv", log_prefix.c_str());
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/car/control", 10);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) {path_ = av::pointsFromPath(*msg);});
    obs_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/obstacles", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        obstacles_ = av::obstaclesFromArray(*msg);
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {onOdometry(*msg);});
  }

private:
  av::FollowerParams loadParams()
  {
    av::FollowerParams fp;
    fp.limits.wheelbase = declare_parameter<double>("wheelbase", fp.limits.wheelbase);
    fp.limits.steer_max = declare_parameter<double>("steer_max", fp.limits.steer_max);
    fp.limits.steer_rate_max = declare_parameter<double>("steer_rate_max", fp.limits.steer_rate_max);
    fp.limits.a_min = declare_parameter<double>("a_min", fp.limits.a_min);
    fp.limits.a_max = declare_parameter<double>("a_max", fp.limits.a_max);
    fp.limits.v_max = declare_parameter<double>("v_max", fp.limits.v_max);
    fp.cruise_speed = declare_parameter<double>("cruise_speed", fp.cruise_speed);
    fp.lookahead_min = declare_parameter<double>("lookahead_min", fp.lookahead_min);
    fp.lookahead_gain = declare_parameter<double>("lookahead_gain", fp.lookahead_gain);
    fp.min_speed_factor = declare_parameter<double>("min_speed_factor", fp.min_speed_factor);
    fp.goal_slowdown_dist = declare_parameter<double>("goal_slowdown_dist", fp.goal_slowdown_dist);
    fp.goal_stop_tol = declare_parameter<double>("goal_tolerance", fp.goal_stop_tol);

    fp.speed.kp = declare_parameter<double>("speed_pid.kp", fp.speed.kp);
    fp.speed.ki = declare_parameter<double>("speed_pid.ki", fp.speed.ki);
    fp.speed.kd = declare_parameter<double>("speed_pid.kd", fp.speed.kd);
    fp.speed.i_limit = declare_parameter<double>("speed_pid.i_limit", fp.speed.i_limit);
    fp.heading.kp = declare_parameter<double>("heading_pid.kp", fp.heading.kp);
    fp.heading.ki = declare_parameter<double>("heading_pid.ki", fp.heading.ki);
    fp.heading.kd = declare_parameter<double>("heading_pid.kd", fp.heading.kd);
    fp.heading.i_limit = declare_parameter<double>("heading_pid.i_limit", fp.heading.i_limit);

    const auto goal = declare_parameter<std::vector<double>>("goal", {25.0, 2.0});
    if (goal.size() >= 2) {
      goal_ = {goal[0], goal[1]};
    }
    return fp;
  }

  void onOdometry(const nav_msgs::msg::Odometry & msg)
  {
    const double t = rclcpp::Time(msg.header.stamp).seconds();
    const av::CarState s = av::carStateFromOdometry(msg);

    // Wait for the first path, then start the clock (t = 0 like the Python sim).
    if (!started_) {
      if (path_.size() < 2) {
        return;
      }
      started_ = true;
      t0_ = t;
      last_t_ = t - control_period_;
      v_cmd_ = std::max(0.0, s.v);
      RCLCPP_INFO(get_logger(), "Path received -> controller running");
    }
    if (t < last_t_) {        // simulation reset
      last_t_ = t - control_period_;
    }
    const double dt = t - last_t_;
    if (dt < control_period_ - 1e-6) {
      return;
    }
    last_t_ = t;

    const auto & lim = follower_.params().limits;
    const av::ControlOutput u = follower_.compute(s, path_, goal_, dt);

    // Kinematic bicycle model -> Gazebo velocity command
    v_cmd_ = std::clamp(v_cmd_ + u.a * dt, 0.0, lim.v_max);
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v_cmd_;
    cmd.angular.z = v_cmd_ * std::tan(u.delta) / lim.wheelbase;
    cmd_pub_->publish(cmd);

    std_msgs::msg::Float64MultiArray dbg;
    dbg.data = {u.a, u.delta, v_cmd_, u.v_target};
    debug_pub_->publish(dbg);

    if (u.goal_reached && v_cmd_ <= 1e-3 && !goal_reported_) {
      RCLCPP_INFO(get_logger(), "t=%.1f goal reached, car stopped", t - t0_);
      goal_reported_ = true;
    }
    writeLog(t - t0_, s, u);
  }

  void writeLog(double t, const av::CarState & s, const av::ControlOutput & u)
  {
    if (!log_.is_open() || obstacles_.empty()) {
      return;
    }
    if (!log_header_written_) {
      log_ << "t,x,y,psi,v,a,delta,v_target";
      for (size_t j = 0; j < obstacles_.size(); ++j) {
        log_ << ",o" << j << "x,o" << j << "y";
      }
      log_ << "\n";
      log_header_written_ = true;
    }
    log_ << t << "," << s.x << "," << s.y << "," << s.psi << "," << s.v << ","
         << u.a << "," << u.delta << "," << u.v_target;
    for (const auto & o : obstacles_) {
      log_ << "," << o.x << "," << o.y;
    }
    log_ << "\n";
    log_.flush();
  }

  // goal_ must be declared before follower_: follower_ is built from
  // loadParams(), which also sets goal_ (members initialise in this order).
  av::Point2 goal_{25.0, 2.0};
  av::PathFollower follower_;
  std::vector<av::Point2> path_;
  std::vector<av::Obstacle> obstacles_;
  double control_period_{0.1};
  double v_cmd_{0.0};
  double t0_{0.0};
  double last_t_{0.0};
  bool started_{false};
  bool goal_reported_{false};
  bool log_header_written_{false};
  std::ofstream log_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obs_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDControllerNode>());
  rclcpp::shutdown();
  return 0;
}
