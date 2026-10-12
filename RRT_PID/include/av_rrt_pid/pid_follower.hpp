// PID path follower -- the replacement for the NMPC.
//
// Two independent PID loops produce the same inputs the NMPC produced,
// u = [a, delta], with the same bounds:
//   speed loop   : e_v   = v_target - v                -> a     in [a_min, a_max]
//   heading loop : e_psi = bearing to lookahead - psi  -> delta in [-pi/10, pi/10]
// The lookahead point sits Ld = lookahead_min + lookahead_gain * v metres
// further along the RRT path than the car's closest point on it.
#pragma once

#include <vector>

#include "av_rrt_pid/bicycle_model.hpp"
#include "av_rrt_pid/types.hpp"

namespace av_rrt_pid
{

struct PIDGains
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double i_limit{1.0};  // clamp on the integral term (anti-windup)
};

class PID
{
public:
  PID() = default;
  explicit PID(const PIDGains & g) : g_(g) {}

  void setGains(const PIDGains & g) {g_ = g;}
  void reset();

  // Returns the clamped PID output.
  //  * Derivative on measurement: the D-term uses -d(measurement)/dt, not
  //    d(error)/dt, so a jump in the setpoint (e.g. a new RRT path) does not
  //    produce a "derivative kick". Pass the measurement's rate of change.
  //  * Conditional integration: the integral stops growing while the output
  //    is saturated in the direction of the error (anti-windup).
  double update(double error, double measurement_rate, double dt, double out_min, double out_max);

private:
  PIDGains g_;
  double integral_{0.0};
};

struct FollowerParams
{
  VehicleLimits limits;
  double cruise_speed{1.5};         // [m/s]
  double lookahead_min{0.8};        // [m]
  double lookahead_gain{0.3};       // [s]
  double min_speed_factor{0.4};     // slow down for large heading errors
  double goal_slowdown_dist{4.0};   // [m] start slowing this far from the final goal
  double goal_stop_tol{0.5};        // [m]
  // Both plants are integrators in the kinematic bicycle model
  // (v' = a, psi' = v tan(delta) / L respond instantly to the input), so
  // P + I does the work. kd is available, but a D-term on the yaw rate feeds
  // back last step's steering and makes delta chatter -- start from 0.
  PIDGains speed{1.5, 0.3, 0.0, 1.0};
  PIDGains heading{2.0, 0.05, 0.0, 0.3};
};

struct ControlOutput
{
  double a{0.0};          // [m/s^2]
  double delta{0.0};      // [rad]
  double v_target{0.0};   // [m/s]
  double heading_error{0.0};
  Point2 lookahead;
  bool goal_reached{false};
};

class PathFollower
{
public:
  explicit PathFollower(const FollowerParams & p);

  ControlOutput compute(
    const CarState & s, const std::vector<Point2> & path,
    const Point2 & final_goal, double dt);

  void reset();
  const FollowerParams & params() const {return p_;}

private:
  FollowerParams p_;
  PID speed_pid_;
  PID heading_pid_;
  CarState prev_;
  bool have_prev_{false};
  double prev_delta_{0.0};
};

}  // namespace av_rrt_pid
