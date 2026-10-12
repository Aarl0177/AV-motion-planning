// Time-aware, turn-radius-aware RRT for a car among moving obstacles.
//
// Time:  every tree node is reached at t = timeAt(path length from the root),
//        assuming the car accelerates from its current speed to plan_speed.
//        Edges are collision-checked against each obstacle's constant-velocity
//        prediction at that time -- the same prediction the NMPC used.
//
// Turns: plain RRT ignores that the car cannot turn tighter than
//        R_min = L / tan(steer_max) (~3.1 m here). A corner of angle theta
//        between two segments of lengths l1, l2 can be rounded by an arc of
//        radius R_min only if  R_min * tan(theta / 2) <= min(l1, l2) / 2.
//        Tree growth and shortcutting both enforce this, so every path the
//        PID is asked to follow is drivable.
#pragma once

#include <random>
#include <vector>

#include "av_rrt_pid/types.hpp"

namespace av_rrt_pid
{

struct RRTParams
{
  // Tree growth
  double step_size{0.6};          // [m] max edge length when extending
  double goal_bias{0.15};         // probability of sampling the goal
  double connect_radius{3.0};     // try a straight connection to the goal within this distance
  int max_iterations{3000};
  double sample_margin{3.0};      // [m] sample x in [min(start,goal)-m, max(start,goal)+m]

  // Workspace (road) bounds
  double x_min{-5.0};
  double x_max{40.0};
  double y_min{-1.5};
  double y_max{5.5};

  // Safety: same numbers as the Python version
  double obs_radius{0.30};
  double vehicle_radius{0.20};
  double margin{0.40};            // safe_radius = obs + vehicle + margin = 0.9 m

  // Time model
  double plan_speed{1.5};         // [m/s] cruise speed used to time-stamp the path
  double plan_accel{1.0};         // [m/s^2] assumed acceleration from the current speed
  double prediction_horizon{6.0}; // [s] predictions are frozen after this time
  double check_resolution{0.1};   // [m] spacing of collision checks along an edge

  // Kinematics
  double min_turn_radius{3.08};   // [m] L / tan(steer_max) = 1 / tan(pi/10)
  double root_virtual_length{1.2};// [m] length of the imaginary segment behind the car

  // If the goal itself cannot be reached, return the branch that gets
  // closest to it, provided it makes at least this much progress [m].
  // (Receding-horizon behaviour: drive towards the goal and replan later.)
  bool allow_partial_path{true};
  double min_partial_progress{2.0};

  // Post-processing
  bool shortcut{true};
  int smoothing_iterations{2};    // Chaikin corner-cutting passes (0 = off)

  unsigned int seed{0};           // 0 = random seed

  double safeRadius() const {return obs_radius + vehicle_radius + margin;}
  double hardRadius() const {return obs_radius + vehicle_radius;}
};

struct RRTNode
{
  Point2 p;
  int parent{-1};
  double cost{0.0};     // path length from the root
  double heading{0.0};  // direction of the incoming edge (car heading at the root)
  double in_len{0.0};   // length of the incoming edge
};

class RRTPlanner
{
public:
  explicit RRTPlanner(const RRTParams & params);

  // Current speed of the car; used to time-stamp paths (call before plan()).
  void setStartSpeed(double v) {v_start_ = v;}

  // Time [s] at which the car reaches arc length s along a path from now.
  double timeAt(double s) const;

  // Plan from `start` (car heading `start_heading`) to `goal`.
  // Obstacles are given at their current position (t = 0).
  // `inflation` is extra clearance added on top of safe_radius while planning.
  bool plan(
    const Point2 & start, double start_heading, const Point2 & goal,
    const std::vector<Obstacle> & obstacles, double inflation,
    std::vector<Point2> & path_out);

  // Check a path that starts at the car *now* against the predictions.
  bool isPathFree(
    const std::vector<Point2> & path, const std::vector<Obstacle> & obstacles,
    double inflation = 0.0) const;

  // Is `p` free during the window of times the car could arrive there?
  bool isGoalFree(
    const Point2 & start, const Point2 & p, const std::vector<Obstacle> & obstacles,
    double inflation) const;

  const std::vector<RRTNode> & tree() const {return tree_;}
  const RRTParams & params() const {return p_;}
  bool lastPlanPartial() const {return last_partial_;}

private:
  std::vector<double> requiredClearance(
    const Point2 & start, const std::vector<Obstacle> & obstacles, double inflation) const;
  bool pointFree(
    const Point2 & q, double t, const std::vector<Obstacle> & obstacles,
    const std::vector<double> & required) const;
  bool segmentFree(
    const Point2 & a, const Point2 & b, double s_a,
    const std::vector<Obstacle> & obstacles, const std::vector<double> & required) const;
  int nearest(const Point2 & q) const;

  // Largest corner angle that an arc of min_turn_radius can round between
  // segments of length l_in and l_out.
  double maxTurn(double l_in, double l_out) const;
  bool turnOk(double heading_in, double l_in, const Point2 & from, const Point2 & to) const;

  // Extend from node `n` towards `target`, rotating the direction into the
  // allowed cone if necessary.
  Point2 steer(const RRTNode & n, const Point2 & target) const;

  std::vector<Point2> shortcutPath(
    const std::vector<Point2> & raw, double start_heading,
    const std::vector<Obstacle> & obstacles, const std::vector<double> & required) const;
  std::vector<Point2> chaikin(const std::vector<Point2> & path) const;

  RRTParams p_;
  std::vector<RRTNode> tree_;
  std::mt19937 rng_;
  bool last_partial_{false};
  double v_start_{0.0};
};

}  // namespace av_rrt_pid
