// Basic types shared by the planner, the controller and the ROS nodes.
// Nothing in here depends on ROS, so the same code runs in the
// standalone simulator and inside the ROS 2 nodes.
#pragma once

#include <cmath>
#include <vector>

namespace av_rrt_pid
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

// Same state as the Python version: [x, y, psi, v]
struct CarState
{
  double x{0.0};
  double y{0.0};
  double psi{0.0};
  double v{0.0};
};

// Same obstacle state as the Python version: [ox, oy, ovx, ovy]
struct Obstacle
{
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};

  // Constant-velocity prediction (identical to AV.predict_obstacle)
  Point2 predict(double t) const {return {x + vx * t, y + vy * t};}
};

inline double wrapAngle(double a) {return std::atan2(std::sin(a), std::cos(a));}

inline double distance(const Point2 & a, const Point2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

inline double pathLength(const std::vector<Point2> & path)
{
  double len = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    len += distance(path[i - 1], path[i]);
  }
  return len;
}

// Goal test for a car driving in +x along a lane: inside the tolerance
// circle, or already across the goal line (x >= goal.x) near the lane.
// Without the second condition a car that coasts slightly past the goal
// would try to plan back to it, which it cannot do without reversing.
inline bool goalReached(const Point2 & p, const Point2 & goal, double tol)
{
  return distance(p, goal) < tol || (p.x >= goal.x && std::fabs(p.y - goal.y) < 2.0 * tol);
}

// Closest point of a polyline to p.
struct Projection
{
  size_t segment{0};     // index i of segment [i, i+1]
  Point2 point;          // closest point on the polyline
  double arc_length{0};  // arc length from path start to `point`
  double distance{0};    // distance from p to `point`
};

inline Projection projectOntoPath(const std::vector<Point2> & path, const Point2 & p)
{
  Projection best;
  best.distance = 1e18;
  if (path.empty()) {
    return best;
  }
  if (path.size() == 1) {
    best.point = path[0];
    best.distance = distance(p, path[0]);
    return best;
  }
  double s_before = 0.0;
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const Point2 & a = path[i];
    const Point2 & b = path[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    double u = 0.0;
    if (len2 > 1e-12) {
      u = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
      u = std::fmin(1.0, std::fmax(0.0, u));
    }
    const Point2 q{a.x + u * dx, a.y + u * dy};
    const double d = distance(p, q);
    if (d < best.distance) {
      best.segment = i;
      best.point = q;
      best.distance = d;
      best.arc_length = s_before + u * std::sqrt(len2);
    }
    s_before += std::sqrt(len2);
  }
  return best;
}

// Point at arc length s along the polyline (clamped to the end points).
inline Point2 pointAtArcLength(const std::vector<Point2> & path, double s)
{
  if (path.empty()) {
    return {};
  }
  if (s <= 0.0) {
    return path.front();
  }
  for (size_t i = 0; i + 1 < path.size(); ++i) {
    const double seg = distance(path[i], path[i + 1]);
    if (s <= seg && seg > 1e-12) {
      const double u = s / seg;
      return {path[i].x + u * (path[i + 1].x - path[i].x),
        path[i].y + u * (path[i + 1].y - path[i].y)};
    }
    s -= seg;
  }
  return path.back();
}

}  // namespace av_rrt_pid
