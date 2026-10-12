// Kinematic bicycle model -- the same equations as AV.dynamics() in the
// Python/CasADi version. Used by the standalone simulator. In Gazebo the
// same model is realised by sending v and omega = v * tan(delta) / L to
// the car's VelocityControl plugin.
#pragma once

#include <algorithm>
#include <cmath>

#include "av_rrt_pid/types.hpp"

namespace av_rrt_pid
{

struct VehicleLimits
{
  double wheelbase{1.0};          // L
  double a_min{-2.0};             // [m/s^2]
  double a_max{2.0};
  double steer_max{M_PI / 10.0};  // |delta| <= pi/10
  double steer_rate_max{1.5};     // [rad/s] steering actuator rate limit (<= 0 disables)
  double v_max{3.0};              // forward-only, [0, v_max]
};

inline CarState bicycleStep(const CarState & s, double a, double delta, double dt, double L)
{
  CarState n;
  n.x = s.x + dt * s.v * std::cos(s.psi);
  n.y = s.y + dt * s.v * std::sin(s.psi);
  n.psi = wrapAngle(s.psi + dt * s.v / L * std::tan(delta));
  n.v = s.v + dt * a;
  return n;
}

// Yaw rate of the kinematic bicycle model for a given speed and steering angle.
inline double bicycleYawRate(double v, double delta, double L)
{
  return v * std::tan(delta) / L;
}

}  // namespace av_rrt_pid
