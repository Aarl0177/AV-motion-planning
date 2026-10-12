#include "av_rrt_pid/world_sdf.hpp"

#include <cmath>
#include <sstream>
#include <string>

namespace av_rrt_pid
{
namespace
{

// Number -> text without trailing noise ("0.3", "1.5708", "25").
std::string num(double v)
{
  std::ostringstream os;
  os.precision(6);
  os << v;
  return os.str();
}

std::string vec3(double a, double b, double c)
{
  return num(a) + " " + num(b) + " " + num(c);
}

std::string pose(double x, double y, double z, double roll, double pitch, double yaw)
{
  return vec3(x, y, z) + " " + vec3(roll, pitch, yaw);
}

struct Rgb
{
  double r, g, b;
};

std::string material(const Rgb & c, double a = 1.0)
{
  const std::string rgba = vec3(c.r, c.g, c.b) + " " + num(a);
  return "<material><ambient>" + rgba + "</ambient><diffuse>" + rgba +
         "</diffuse><specular>0.1 0.1 0.1 1</specular></material>";
}

std::string boxVisual(
  const std::string & name, const std::string & p, const std::string & size,
  const Rgb & c, double transparency = 0.0)
{
  std::ostringstream os;
  os << "\n        <visual name=\"" << name << "\">"
     << "\n          <pose>" << p << "</pose>"
     << "\n          <geometry><box><size>" << size << "</size></box></geometry>"
     << "\n          " << material(c)
     << "\n          <transparency>" << num(transparency) << "</transparency>"
     << "\n        </visual>";
  return os.str();
}

std::string cylinderVisual(
  const std::string & name, const std::string & p, double radius, double length,
  const Rgb & c, double transparency = 0.0, bool shadows = true)
{
  std::ostringstream os;
  os << "\n        <visual name=\"" << name << "\">"
     << "\n          <pose>" << p << "</pose>"
     << "\n          <geometry><cylinder><radius>" << num(radius) << "</radius><length>"
     << num(length) << "</length></cylinder></geometry>"
     << "\n          " << material(c)
     << "\n          <transparency>" << num(transparency) << "</transparency>"
     << "\n          <cast_shadows>" << (shadows ? "true" : "false") << "</cast_shadows>"
     << "\n        </visual>";
  return os.str();
}

// VelocityControl listens on /model/<name>/cmd_vel (gz.msgs.Twist, body frame).
// OdometryPublisher publishes /model/<name>/odometry (world pose, body twist).
std::string velocityAndOdometryPlugins(const std::string & model_name)
{
  return "\n      <plugin filename=\"gz-sim-velocity-control-system\""
         "\n              name=\"gz::sim::systems::VelocityControl\"/>"
         "\n      <plugin filename=\"gz-sim-odometry-publisher-system\""
         "\n              name=\"gz::sim::systems::OdometryPublisher\">"
         "\n        <odom_frame>world</odom_frame>"
         "\n        <robot_base_frame>" + model_name + "</robot_base_frame>"
         "\n        <odom_publish_frequency>50</odom_publish_frequency>"
         "\n        <dimensions>2</dimensions>"
         "\n      </plugin>";
}

// LiDAR beam from p0 to p1 (MuJoCo capsule 'fromto') as a cylinder:
// the cylinder axis is z, so pitch it by 90 deg onto x, then yaw it.
std::string beam(const std::string & name, const double p0[3], const double p1[3])
{
  const double dx = p1[0] - p0[0];
  const double dy = p1[1] - p0[1];
  const double dz = p1[2] - p0[2];
  const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
  const double yaw = std::atan2(dy, dx);
  const std::string p = pose(
    (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, (p0[2] + p1[2]) / 2, 0.0, M_PI / 2, yaw);
  return cylinderVisual(name, p, 0.012, length, {0.1, 0.8, 1.0}, 0.65, false);
}

std::string carModel(const WorldSpec & s)
{
  const Rgb wheel{0.01, 0.01, 0.01};
  struct W {const char * name; double x, y;};
  const W wheels[] = {
    {"front_left", 0.23, 0.21}, {"front_right", 0.23, -0.21},
    {"rear_left", -0.23, 0.21}, {"rear_right", -0.23, -0.21}};
  const double l0[3] = {0.39, 0.02, 0.08}, l1[3] = {3.20, 0.80, 0.08};
  const double r0[3] = {0.39, -0.02, 0.08}, r1[3] = {3.20, -0.80, 0.08};

  std::ostringstream os;
  os << "\n    <model name=\"car\">"
     << "\n      <pose>" << pose(s.start_x, s.start_y, 0.13, 0, 0, s.start_yaw) << "</pose>"
     << "\n      <link name=\"base_link\">"
     << "\n        <inertial>"
     << "\n          <mass>10.0</mass>"
     << "\n          <inertia><ixx>0.15</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.45</iyy>"
     << "<iyz>0</iyz><izz>0.5</izz></inertia>"
     << "\n        </inertial>"
     << "\n        <collision name=\"chassis_collision\">"
     << "\n          <geometry><box><size>0.7 0.36 0.16</size></box></geometry>"
     << "\n        </collision>"
     << boxVisual("chassis", "0 0 0 0 0 0", "0.7 0.36 0.16", {0.1, 0.25, 0.9})
     << boxVisual("hood", "0.22 0 0.03 0 0 0", "0.24 0.30 0.10", {0.2, 0.48, 1.0})
     << boxVisual("cabin", "-0.06 0 0.11 0 0 0", "0.28 0.26 0.14", {0.03, 0.05, 0.12})
     << boxVisual("lidar_sensor", "0.38 0 0.08 0 0 0", "0.05 0.10 0.05", {0.0, 0.0, 0.0})
     << beam("lidar_left_beam", l0, l1)
     << beam("lidar_right_beam", r0, r1);
  for (const auto & w : wheels) {
    // cylinder axis z -> roll 90 deg puts it along y, like a wheel axle
    os << cylinderVisual(
      std::string(w.name) + "_wheel", pose(w.x, w.y, -0.07, M_PI / 2, 0, 0), 0.055, 0.05, wheel);
  }
  os << "\n      </link>"
     << velocityAndOdometryPlugins("car")
     << "\n    </model>";
  return os.str();
}

std::string obstacleModel(size_t j, const Obstacle & o, double radius, double safe_radius)
{
  const std::string name = "obs_" + std::to_string(j);
  const double z = radius + 0.01;  // float just above the ground
  const double mass = 5.0;
  const std::string I = num(0.4 * mass * radius * radius);  // solid sphere
  std::ostringstream os;
  os << "\n    <model name=\"" << name << "\">"
     << "\n      <pose>" << pose(o.x, o.y, z, 0, 0, 0) << "</pose>"
     << "\n      <link name=\"link\">"
     << "\n        <inertial>"
     << "\n          <mass>" << num(mass) << "</mass>"
     << "\n          <inertia><ixx>" << I << "</ixx><ixy>0</ixy><ixz>0</ixz><iyy>" << I
     << "</iyy><iyz>0</iyz><izz>" << I << "</izz></inertia>"
     << "\n        </inertial>"
     << "\n        <collision name=\"collision\">"
     << "\n          <geometry><sphere><radius>" << num(radius) << "</radius></sphere></geometry>"
     << "\n        </collision>"
     << "\n        <visual name=\"visual\">"
     << "\n          <geometry><sphere><radius>" << num(radius) << "</radius></sphere></geometry>"
     << "\n          " << material({0.9, 0.1, 0.1})
     << "\n        </visual>"
     << cylinderVisual(
    "safe_zone", pose(0, 0, -(z - 0.015), 0, 0, 0), safe_radius, 0.02,
    {1.0, 0.0, 0.0}, 0.82, false)
     << "\n      </link>"
     << velocityAndOdometryPlugins(name)
     << "\n    </model>";
  return os.str();
}

}  // namespace

std::string buildWorldSdf(const WorldSpec & s)
{
  const double lane_len = 50.0;
  std::ostringstream os;
  os << "<?xml version=\"1.0\"?>\n"
     << "<!-- Generated by make_world from config/params.yaml. -->\n"
     << "<sdf version=\"1.9\">\n"
     << "  <world name=\"" << s.world_name << "\">\n"
     << R"(
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- The car and the obstacles are velocity-controlled kinematic bodies
         (like the MuJoCo replay, where the bicycle model set the body pose).
         With gravity on they would slowly sink between velocity updates. -->
    <gravity>0 0 0</gravity>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.80 0.85 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>80 20</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>80 20</size></plane></geometry>
          )" << material({0.28, 0.28, 0.28}) << R"(
        </visual>
      </link>
    </model>
)"
     << "\n    <!-- Reference lane y = " << num(s.lane_y) << " (visual only) -->"
     << "\n    <model name=\"reference_lane\">"
     << "\n      <static>true</static>"
     << "\n      <pose>" << pose(s.goal.x / 2.0, s.lane_y, 0.005, 0, 0, 0) << "</pose>"
     << "\n      <link name=\"link\">"
     << boxVisual("lane", "0 0 0 0 0 0", num(lane_len) + " 0.05 0.01", {0.1, 0.8, 0.1})
     << "\n      </link>"
     << "\n    </model>\n"
     << "\n    <!-- Goal marker (visual only) -->"
     << "\n    <model name=\"goal_marker\">"
     << "\n      <static>true</static>"
     << "\n      <pose>" << pose(s.goal.x, s.goal.y, 0.01, 0, 0, 0) << "</pose>"
     << "\n      <link name=\"link\">"
     << cylinderVisual("goal", "0 0 0 0 0 0", 0.5, 0.02, {0.1, 0.9, 0.2}, 0.5, false)
     << "\n      </link>"
     << "\n    </model>\n"
     << carModel(s) << "\n";
  for (size_t j = 0; j < s.obstacles.size(); ++j) {
    os << obstacleModel(j, s.obstacles[j], s.obs_radius, s.safe_radius) << "\n";
  }
  os << "\n  </world>\n</sdf>\n";
  return os.str();
}

std::string buildBridgeConfig(size_t num_obstacles)
{
  // ros_gz_bridge 'config_file' format. GZ_TO_ROS = Gazebo -> ROS.
  auto entry = [](const std::string & topic, const std::string & ros_type,
    const std::string & gz_type, const std::string & dir) {
      return "- ros_topic_name: \"" + topic + "\"\n"
             "  gz_topic_name: \"" + topic + "\"\n"
             "  ros_type_name: \"" + ros_type + "\"\n"
             "  gz_type_name: \"" + gz_type + "\"\n"
             "  direction: " + dir + "\n";
    };
  std::string y = "# Generated by make_world. ROS <-> Gazebo topics for ros_gz_bridge.\n";
  y += entry("/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", "GZ_TO_ROS");
  y += entry("/model/car/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist", "ROS_TO_GZ");
  y += entry("/model/car/odometry", "nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS");
  for (size_t j = 0; j < num_obstacles; ++j) {
    const std::string base = "/model/obs_" + std::to_string(j);
    y += entry(base + "/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist", "ROS_TO_GZ");
    y += entry(base + "/odometry", "nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS");
  }
  return y;
}

}  // namespace av_rrt_pid
