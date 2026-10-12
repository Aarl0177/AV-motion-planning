#include "av_rrt_pid/local_planner.hpp"

#include <algorithm>
#include <cmath>

namespace av_rrt_pid
{

LocalPlanner::LocalPlanner(const LocalPlannerParams & lp, const RRTParams & rrt)
: lp_(lp), rrt_(rrt)
{
}

// Local goal on the lane, `goal_lookahead` ahead of the car. If that point
// will be occupied when the car gets there, slide it along the lane.
Point2 LocalPlanner::chooseLocalGoal(
  const CarState & car, const std::vector<Obstacle> & obstacles) const
{
  const Point2 pos{car.x, car.y};
  const double gx = std::min(car.x + lp_.goal_lookahead, lp_.final_goal.x);
  const bool at_final = gx >= lp_.final_goal.x - 1e-9;
  const Point2 nominal = at_final ? lp_.final_goal : Point2{gx, lp_.lane_y};

  if (rrt_.isGoalFree(pos, nominal, obstacles, lp_.plan_inflation)) {
    return nominal;
  }
  const double offsets[] = {1.0, -1.0, 2.0, -2.0, 3.0, -3.0, 4.0};
  for (double dx : offsets) {
    const Point2 g{gx + dx, lp_.lane_y};
    if (g.x > lp_.final_goal.x || g.x < car.x + 1.5) {
      continue;
    }
    if (rrt_.isGoalFree(pos, g, obstacles, lp_.plan_inflation)) {
      return g;
    }
  }
  return nominal;  // nothing free: let the RRT try (and probably fail -> brake)
}

LocalPlanner::Status LocalPlanner::update(
  const CarState & car, const std::vector<Obstacle> & obstacles)
{
  const Point2 pos{car.x, car.y};
  rrt_.setStartSpeed(car.v);

  if (goalReached(pos, lp_.final_goal, lp_.goal_reached_tol)) {
    reason_ = "goal reached";
    return Status::kGoalReached;
  }

  // ---- Is the current path still good? ----
  reason_.clear();
  if (path_.empty()) {
    reason_ = "no path";
  } else {
    const Projection proj = projectOntoPath(path_, pos);
    std::vector<Point2> remaining{pos};
    remaining.insert(remaining.end(), path_.begin() + proj.segment + 1, path_.end());

    const bool ends_at_final = distance(path_.back(), lp_.final_goal) < 1e-3;
    if (proj.distance > lp_.max_deviation) {
      reason_ = "deviation";
    } else if (!rrt_.isPathFree(remaining, obstacles, 0.0)) {
      reason_ = "collision predicted";
    } else if (!ends_at_final && pathLength(remaining) < lp_.replan_remaining) {
      reason_ = "horizon";
    }
  }

  if (reason_.empty()) {
    return Status::kKeep;
  }

  // ---- Replan ----
  local_goal_ = chooseLocalGoal(car, obstacles);
  std::vector<Point2> new_path;
  if (rrt_.plan(pos, car.psi, local_goal_, obstacles, lp_.plan_inflation, new_path)) {
    path_ = new_path;
    if (rrt_.lastPlanPartial()) {
      reason_ += " (partial path)";
    }
    return Status::kReplanned;
  }

  // Failed. If the old path is merely short (still safe) keep driving on it,
  // otherwise drop it so the controller brakes.
  if (reason_ != "horizon") {
    path_.clear();
  }
  reason_ += " -> RRT failed";
  return Status::kFailed;
}

}  // namespace av_rrt_pid
