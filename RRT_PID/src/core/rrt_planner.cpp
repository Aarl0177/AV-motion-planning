#include "av_rrt_pid/rrt_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace av_rrt_pid
{

RRTPlanner::RRTPlanner(const RRTParams & params)
: p_(params),
  rng_(params.seed != 0 ? params.seed : std::random_device{}())
{
}

// ---------------------------------------------------------------------------
// Time model and collision checking
// ---------------------------------------------------------------------------

// Speed profile: accelerate from the current speed at plan_accel, then cruise.
double RRTPlanner::timeAt(double s) const
{
  const double vp = p_.plan_speed;
  const double a = p_.plan_accel;
  const double v0 = std::clamp(v_start_, 0.0, vp);
  const double s_acc = (vp * vp - v0 * v0) / (2.0 * a);
  if (s <= s_acc) {
    return (-v0 + std::sqrt(v0 * v0 + 2.0 * a * s)) / a;
  }
  return (vp - v0) / a + (s - s_acc) / vp;
}

// Clearance each obstacle needs. Normally safe_radius + inflation; if the car
// is already inside that zone (e.g. an obstacle drifted towards it) we only
// ask it not to get any closer, so the planner can still find a way out.
std::vector<double> RRTPlanner::requiredClearance(
  const Point2 & start, const std::vector<Obstacle> & obstacles, double inflation) const
{
  const double r = p_.safeRadius() + inflation;
  std::vector<double> req(obstacles.size(), r);
  for (size_t j = 0; j < obstacles.size(); ++j) {
    const double d0 = distance(start, obstacles[j].predict(0.0));
    if (d0 < r) {
      req[j] = std::max(p_.hardRadius(), d0 - 0.05);
    }
  }
  return req;
}

bool RRTPlanner::pointFree(
  const Point2 & q, double t, const std::vector<Obstacle> & obstacles,
  const std::vector<double> & required) const
{
  if (q.x < p_.x_min || q.x > p_.x_max || q.y < p_.y_min || q.y > p_.y_max) {
    return false;
  }
  const double tc = std::min(t, p_.prediction_horizon);
  for (size_t j = 0; j < obstacles.size(); ++j) {
    if (distance(q, obstacles[j].predict(tc)) < required[j]) {
      return false;
    }
  }
  return true;
}

// Walk along the edge a->b, which starts at arc length s_a from the car.
// Each point is checked against the obstacles' predicted positions at the
// time the car gets there.
bool RRTPlanner::segmentFree(
  const Point2 & a, const Point2 & b, double s_a,
  const std::vector<Obstacle> & obstacles, const std::vector<double> & required) const
{
  const double len = distance(a, b);
  const int n = std::max(1, static_cast<int>(std::ceil(len / p_.check_resolution)));
  for (int i = 1; i <= n; ++i) {
    const double u = static_cast<double>(i) / n;
    const Point2 q{a.x + u * (b.x - a.x), a.y + u * (b.y - a.y)};
    if (!pointFree(q, timeAt(s_a + u * len), obstacles, required)) {
      return false;
    }
  }
  return true;
}

bool RRTPlanner::isPathFree(
  const std::vector<Point2> & path, const std::vector<Obstacle> & obstacles,
  double inflation) const
{
  if (path.empty()) {
    return false;
  }
  const auto required = requiredClearance(path.front(), obstacles, inflation);
  double cost = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    if (!segmentFree(path[i], path[i + 1], cost, obstacles, required)) {
      return false;
    }
    cost += distance(path[i], path[i + 1]);
  }
  return true;
}

// The RRT path will be longer than the straight line and the car may be
// slower than planned, so check a window of arrival times, not a single one.
bool RRTPlanner::isGoalFree(
  const Point2 & start, const Point2 & p, const std::vector<Obstacle> & obstacles,
  double inflation) const
{
  const auto required = requiredClearance(start, obstacles, inflation);
  const double t0 = timeAt(distance(start, p));
  const double t1 = 1.6 * t0 + 1.0;
  for (double t = t0; t <= t1; t += 0.2) {
    if (!pointFree(p, t, obstacles, required)) {
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Turning constraint
// ---------------------------------------------------------------------------

double RRTPlanner::maxTurn(double l_in, double l_out) const
{
  const double l = std::min(l_in, l_out);
  return 2.0 * std::atan(l / (2.0 * p_.min_turn_radius));
}

bool RRTPlanner::turnOk(
  double heading_in, double l_in, const Point2 & from, const Point2 & to) const
{
  const double dir = std::atan2(to.y - from.y, to.x - from.x);
  return std::fabs(wrapAngle(dir - heading_in)) <= maxTurn(l_in, distance(from, to)) + 1e-9;
}

Point2 RRTPlanner::steer(const RRTNode & n, const Point2 & target) const
{
  const double d = distance(n.p, target);
  const double len = std::min(d, p_.step_size);
  double dir = std::atan2(target.y - n.p.y, target.x - n.p.x);
  const double lim = maxTurn(n.in_len, len);
  const double diff = wrapAngle(dir - n.heading);
  if (std::fabs(diff) > lim) {
    dir = n.heading + std::copysign(lim, diff);
  }
  return {n.p.x + len * std::cos(dir), n.p.y + len * std::sin(dir)};
}

int RRTPlanner::nearest(const Point2 & q) const
{
  int best = 0;
  double best_d = std::numeric_limits<double>::max();
  for (size_t i = 0; i < tree_.size(); ++i) {
    const double dx = tree_[i].p.x - q.x;
    const double dy = tree_[i].p.y - q.y;
    const double d = dx * dx + dy * dy;
    if (d < best_d) {
      best_d = d;
      best = static_cast<int>(i);
    }
  }
  return best;
}

// ---------------------------------------------------------------------------
// RRT
// ---------------------------------------------------------------------------

bool RRTPlanner::plan(
  const Point2 & start, double start_heading, const Point2 & goal,
  const std::vector<Obstacle> & obstacles, double inflation,
  std::vector<Point2> & path_out)
{
  tree_.clear();
  tree_.push_back({start, -1, 0.0, start_heading, p_.root_virtual_length});
  const auto required = requiredClearance(start, obstacles, inflation);

  const double xlo = std::max(p_.x_min, std::min(start.x, goal.x) - p_.sample_margin);
  const double xhi = std::min(p_.x_max, std::max(start.x, goal.x) + p_.sample_margin);
  std::uniform_real_distribution<double> ux(xlo, xhi);
  std::uniform_real_distribution<double> uy(p_.y_min, p_.y_max);
  std::uniform_real_distribution<double> u01(0.0, 1.0);

  // Add an edge parent -> q to the tree (caller has checked it).
  auto add = [this](int parent, const Point2 & q) {
      const RRTNode & n = tree_[parent];
      const double len = distance(n.p, q);
      tree_.push_back({q, parent, n.cost + len, std::atan2(q.y - n.p.y, q.x - n.p.x), len});
      return static_cast<int>(tree_.size()) - 1;
    };

  int goal_idx = -1;

  // Cheap first try: drive straight to the goal.
  if (turnOk(tree_[0].heading, tree_[0].in_len, start, goal) &&
    segmentFree(start, goal, 0.0, obstacles, required))
  {
    goal_idx = add(0, goal);
  }

  for (int it = 0; it < p_.max_iterations && goal_idx < 0; ++it) {
    const Point2 sample = (u01(rng_) < p_.goal_bias) ? goal : Point2{ux(rng_), uy(rng_)};
    const int near = nearest(sample);
    const Point2 q_new = steer(tree_[near], sample);
    if (!segmentFree(tree_[near].p, q_new, tree_[near].cost, obstacles, required)) {
      continue;
    }
    const int new_idx = add(near, q_new);

    // Try to close the gap to the goal with one straight, drivable edge.
    const double d_goal = distance(q_new, goal);
    if (d_goal < 1e-6) {
      goal_idx = new_idx;
    } else if (d_goal <= p_.connect_radius &&  // NOLINT
      turnOk(tree_[new_idx].heading, tree_[new_idx].in_len, q_new, goal) &&
      segmentFree(q_new, goal, tree_[new_idx].cost, obstacles, required))
    {
      goal_idx = add(new_idx, goal);
    }
  }

  last_partial_ = false;
  if (goal_idx < 0) {
    if (!p_.allow_partial_path) {
      return false;
    }
    // Branch that ends closest to the goal.
    double best_d = distance(start, goal);
    for (size_t i = 1; i < tree_.size(); ++i) {
      const double d = distance(tree_[i].p, goal);
      if (d < best_d) {
        best_d = d;
        goal_idx = static_cast<int>(i);
      }
    }
    if (goal_idx < 0 || distance(start, goal) - best_d < p_.min_partial_progress) {
      return false;
    }
    last_partial_ = true;
  }

  std::vector<Point2> raw;
  for (int i = goal_idx; i != -1; i = tree_[i].parent) {
    raw.push_back(tree_[i].p);
  }
  std::reverse(raw.begin(), raw.end());

  std::vector<Point2> path = p_.shortcut ?
    shortcutPath(raw, start_heading, obstacles, required) : raw;

  // Round the remaining corners. Keep a pass only if it stays collision free.
  for (int k = 0; k < p_.smoothing_iterations; ++k) {
    std::vector<Point2> smooth = chaikin(path);
    if (!isPathFree(smooth, obstacles, inflation * 0.5)) {
      break;
    }
    path.swap(smooth);
  }

  path_out = path;
  return true;
}

// Greedy "string pulling": from each kept vertex jump to the farthest
// vertex that can be reached with a collision-free AND drivable edge.
std::vector<Point2> RRTPlanner::shortcutPath(
  const std::vector<Point2> & raw, double start_heading,
  const std::vector<Obstacle> & obstacles, const std::vector<double> & required) const
{
  if (raw.size() < 3) {
    return raw;
  }
  std::vector<Point2> out{raw.front()};
  double cost = 0.0;
  double heading = start_heading;
  double in_len = p_.root_virtual_length;
  size_t i = 0;
  while (i + 1 < raw.size()) {
    size_t j = raw.size() - 1;
    for (; j > i + 1; --j) {
      if (turnOk(heading, in_len, raw[i], raw[j]) &&
        segmentFree(raw[i], raw[j], cost, obstacles, required))
      {
        break;
      }
    }
    // j == i + 1 falls back to the original tree edge.
    const double len = distance(raw[i], raw[j]);
    heading = std::atan2(raw[j].y - raw[i].y, raw[j].x - raw[i].x);
    in_len = len;
    cost += len;
    out.push_back(raw[j]);
    i = j;
  }
  return out;
}

// One pass of Chaikin corner cutting; end points are kept.
std::vector<Point2> RRTPlanner::chaikin(const std::vector<Point2> & path) const
{
  if (path.size() < 3) {
    return path;
  }
  std::vector<Point2> out{path.front()};
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const Point2 & a = path[i];
    const Point2 & b = path[i + 1];
    const Point2 q{0.75 * a.x + 0.25 * b.x, 0.75 * a.y + 0.25 * b.y};
    const Point2 r{0.25 * a.x + 0.75 * b.x, 0.25 * a.y + 0.75 * b.y};
    if (i > 0) {
      out.push_back(q);
    }
    if (i + 2 < path.size()) {
      out.push_back(r);
    }
  }
  out.push_back(path.back());
  return out;
}

}  // namespace av_rrt_pid
