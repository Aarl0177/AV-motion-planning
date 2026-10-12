#include "av_rrt_pid/pid_follower.hpp"

#include <algorithm>
#include <cmath>

namespace av_rrt_pid
{

void PID::reset()
{
  integral_ = 0.0;
}

double PID::update(
  double error, double measurement_rate, double dt, double out_min, double out_max)
{
  const double unsat = g_.kp * error + g_.ki * integral_ - g_.kd * measurement_rate;
  const double out = std::clamp(unsat, out_min, out_max);

  // Conditional integration: don't wind up while pushing against a limit.
  const bool saturated_high = unsat > out_max && error > 0.0;
  const bool saturated_low = unsat < out_min && error < 0.0;
  if (!saturated_high && !saturated_low && dt > 0.0) {
    integral_ = std::clamp(integral_ + error * dt, -g_.i_limit, g_.i_limit);
  }
  return out;
}

PathFollower::PathFollower(const FollowerParams & p)
: p_(p), speed_pid_(p.speed), heading_pid_(p.heading)
{
}

void PathFollower::reset()
{
  speed_pid_.reset();
  heading_pid_.reset();
  have_prev_ = false;
  prev_delta_ = 0.0;
}

ControlOutput PathFollower::compute(
  const CarState & s, const std::vector<Point2> & path,
  const Point2 & final_goal, double dt)
{
  ControlOutput out;
  const auto & lim = p_.limits;
  const Point2 pos{s.x, s.y};
  const double d_goal = distance(pos, final_goal);

  const bool at_goal = goalReached(pos, final_goal, p_.goal_stop_tol);

  // Measured rates for the D-terms (derivative on measurement).
  double yaw_rate = 0.0;
  double accel = 0.0;
  if (have_prev_ && dt > 0.0) {
    yaw_rate = wrapAngle(s.psi - prev_.psi) / dt;
    accel = (s.v - prev_.v) / dt;
  }
  prev_ = s;
  have_prev_ = true;

  // ---- No path, or goal reached: brake to a stop, wheels straight ----
  if (path.size() < 2 || at_goal) {
    out.goal_reached = at_goal;
    out.v_target = 0.0;
    out.a = speed_pid_.update(-s.v, accel, dt, lim.a_min, lim.a_max);
    if (s.v <= 0.05) {
      out.a = std::max(out.a, 0.0);  // never command reverse
    }
    out.delta = 0.0;
    prev_delta_ = 0.0;
    heading_pid_.reset();
    return out;
  }

  // ---- Lateral loop: steer towards a lookahead point on the path ----
  const Projection proj = projectOntoPath(path, pos);
  const double Ld = p_.lookahead_min + p_.lookahead_gain * std::max(0.0, s.v);
  out.lookahead = pointAtArcLength(path, proj.arc_length + Ld);
  // If the lookahead collapses onto the path end, keep aiming beyond it.
  if (distance(pos, out.lookahead) < 0.3) {
    out.lookahead = path.back();
  }

  const double bearing = std::atan2(out.lookahead.y - s.y, out.lookahead.x - s.x);
  out.heading_error = wrapAngle(bearing - s.psi);
  // e = bearing - psi, so d(measurement)/dt = yaw rate
  out.delta = heading_pid_.update(
    out.heading_error, yaw_rate, dt, -lim.steer_max, lim.steer_max);
  // Steering actuator rate limit: smooths the step in heading error when the
  // planner switches to a new path.
  if (lim.steer_rate_max > 0.0 && dt > 0.0) {
    const double max_step = lim.steer_rate_max * dt;
    out.delta = std::clamp(out.delta, prev_delta_ - max_step, prev_delta_ + max_step);
  }
  prev_delta_ = out.delta;

  // ---- Longitudinal loop: track a target speed ----
  double v_target = p_.cruise_speed;
  // Slow down while the heading error is large (tight turns).
  v_target *= std::max(p_.min_speed_factor, std::cos(out.heading_error));
  // Slow down near the final goal: v <= sqrt(2 * a_brake * d), a_brake = 0.5 |a_min|
  if (d_goal < p_.goal_slowdown_dist) {
    v_target = std::min(v_target, std::sqrt(2.0 * 0.5 * std::fabs(lim.a_min) * d_goal));
  }
  out.v_target = std::clamp(v_target, 0.0, lim.v_max);
  out.a = speed_pid_.update(out.v_target - s.v, accel, dt, lim.a_min, lim.a_max);
  return out;
}

}  // namespace av_rrt_pid
