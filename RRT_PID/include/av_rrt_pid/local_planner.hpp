// Receding-horizon wrapper around the RRT.
//
// Like the NMPC, which only looked N steps ahead along the y = 2 lane, the
// planner aims at a local goal `goal_lookahead` metres ahead on the lane
// (capped at the final goal). It keeps the current path while it is still
// safe and replans only when:
//   - there is no path yet,
//   - an obstacle's predicted motion now crosses the remaining path,
//   - the car has drifted too far from the path, or
//   - less than `replan_remaining` metres of path are left.
// Replanning only on these events avoids RRT "flicker" (a new random path
// every cycle, switching sides around an obstacle).
#pragma once

#include <string>
#include <vector>

#include "av_rrt_pid/rrt_planner.hpp"
#include "av_rrt_pid/types.hpp"

namespace av_rrt_pid
{

struct LocalPlannerParams
{
  double lane_y{2.0};               // reference lane, as in the NMPC (y_ref = 2)
  Point2 final_goal{25.0, 2.0};
  double goal_lookahead{8.0};       // [m] local goal distance along the lane
  double goal_reached_tol{0.5};     // [m]
  double max_deviation{1.0};        // [m] replan if the car is further than this from the path
  double replan_remaining{3.0};     // [m] extend the plan when less than this is left
  double plan_inflation{0.30};      // [m] extra clearance used only while planning
};

class LocalPlanner
{
public:
  enum class Status { kKeep, kReplanned, kFailed, kGoalReached };

  LocalPlanner(const LocalPlannerParams & lp, const RRTParams & rrt);

  Status update(const CarState & car, const std::vector<Obstacle> & obstacles);

  const std::vector<Point2> & path() const {return path_;}
  const RRTPlanner & rrt() const {return rrt_;}
  const std::string & lastReason() const {return reason_;}
  const Point2 & localGoal() const {return local_goal_;}
  const LocalPlannerParams & params() const {return lp_;}

private:
  Point2 chooseLocalGoal(const CarState & car, const std::vector<Obstacle> & obstacles) const;

  LocalPlannerParams lp_;
  RRTPlanner rrt_;
  std::vector<Point2> path_;
  Point2 local_goal_;
  std::string reason_;
};

}  // namespace av_rrt_pid
