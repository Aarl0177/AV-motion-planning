// Standalone simulation (no ROS, no Gazebo) -- the C++ counterpart of
// run_nmpc_simulation(). It runs exactly the same LocalPlanner (RRT) and
// PathFollower (PID) classes the ROS nodes use, on the same scenario as
// the Python script, and writes CSV logs for the plot_results tool.
//
//   ./standalone_sim [--seed N] [--v0 V] [--tmax T] [--out PREFIX]
//                    [--cruise V] [--inflation M] [--lookahead M] [--lookahead-gain S]
//                    [--kp-heading K] [--kd-heading K] [--kp-speed K] [--steer-rate R]
//
// The tuning flags override the defaults so you can tune gains here, quickly,
// before copying them into config/params.yaml for the Gazebo run.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "av_rrt_pid/bicycle_model.hpp"
#include "av_rrt_pid/local_planner.hpp"
#include "av_rrt_pid/pid_follower.hpp"

using namespace av_rrt_pid;

int main(int argc, char ** argv)
{
  unsigned int seed = 1;
  double v0 = 0.0;        // the Gazebo car starts at rest (the NMPC script started at 2 m/s)
  double t_max = 40.0;
  std::string out = "sim";

  RRTParams rrt_p;
  LocalPlannerParams lp;
  FollowerParams fp;

  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i];
    const double v = std::atof(argv[i + 1]);
    if (k == "--seed") {seed = std::strtoul(argv[i + 1], nullptr, 10);}
    else if (k == "--v0") {v0 = v;}
    else if (k == "--tmax") {t_max = v;}
    else if (k == "--out") {out = argv[i + 1];}
    else if (k == "--cruise") {fp.cruise_speed = v; rrt_p.plan_speed = v;}
    else if (k == "--inflation") {lp.plan_inflation = v;}
    else if (k == "--lookahead") {fp.lookahead_min = v;}
    else if (k == "--lookahead-gain") {fp.lookahead_gain = v;}
    else if (k == "--kp-heading") {fp.heading.kp = v;}
    else if (k == "--kd-heading") {fp.heading.kd = v;}
    else if (k == "--kp-speed") {fp.speed.kp = v;}
    else if (k == "--steer-rate") {fp.limits.steer_rate_max = v;}
    else {std::fprintf(stderr, "unknown option %s\n", k.c_str()); return 1;}
  }
  rrt_p.seed = seed;
  // Same turning limit as the car: R_min = L / tan(steer_max)
  rrt_p.min_turn_radius = fp.limits.wheelbase / std::tan(fp.limits.steer_max);

  const double dt = 0.1;            // same control period as the Python version
  const int plan_every = 2;         // planner runs at 5 Hz

  LocalPlanner planner(lp, rrt_p);
  PathFollower follower(fp);

  CarState car{0.0, 0.0, 0.0, v0};
  std::vector<Obstacle> obs = {
    {3.0, 1.50, 0.30, 0.05},
    {10.0, 2.65, -0.25, 0.04},
    {20.0, 1.65, -0.50, 0.00},
  };

  std::ofstream log(out + "_log.csv");
  std::ofstream paths(out + "_paths.csv");
  log << "t,x,y,psi,v,a,delta,v_target";
  for (size_t j = 0; j < obs.size(); ++j) {log << ",o" << j << "x,o" << j << "y";}
  log << "\n";
  paths << "plan_id,t,reason,x,y\n";

  std::vector<double> min_clear(obs.size(), 1e9);
  int replans = 0, failures = 0;
  double t_goal = -1.0;
  const int steps = static_cast<int>(t_max / dt);

  for (int k = 0; k <= steps; ++k) {
    const double t = k * dt;

    if (k % plan_every == 0) {
      const auto st = planner.update(car, obs);
      if (st == LocalPlanner::Status::kReplanned) {
        for (const auto & p : planner.path()) {
          paths << replans << "," << t << "," << planner.lastReason() << ","
                << p.x << "," << p.y << "\n";
        }
        ++replans;
      } else if (st == LocalPlanner::Status::kFailed) {
        ++failures;
        std::printf("t=%5.1f  planner: %s\n", t, planner.lastReason().c_str());
      }
    }

    const ControlOutput u = follower.compute(car, planner.path(), lp.final_goal, dt);

    log << t << "," << car.x << "," << car.y << "," << car.psi << "," << car.v << ","
        << u.a << "," << u.delta << "," << u.v_target;
    for (const auto & o : obs) {log << "," << o.x << "," << o.y;}
    log << "\n";

    for (size_t j = 0; j < obs.size(); ++j) {
      min_clear[j] = std::min(min_clear[j], distance({car.x, car.y}, {obs[j].x, obs[j].y}));
    }
    if (u.goal_reached && car.v < 0.05 && t_goal < 0) {
      t_goal = t;
      break;
    }

    car = bicycleStep(car, u.a, u.delta, dt, fp.limits.wheelbase);
    car.v = std::clamp(car.v, 0.0, fp.limits.v_max);
    for (auto & o : obs) {
      o.x += dt * o.vx;   // obstacle_dynamics_all()
      o.y += dt * o.vy;
    }
  }

  std::printf("seed %u | replans %d | planner failures %d | goal %s",
    seed, replans, failures, t_goal >= 0 ? "reached" : "NOT reached");
  if (t_goal >= 0) {std::printf(" at t=%.1f s", t_goal);}
  std::printf("\nmin centre distance to obstacles (safe radius %.2f, collision < %.2f):",
    rrt_p.safeRadius(), rrt_p.hardRadius());
  for (double c : min_clear) {std::printf(" %.3f", c);}
  std::printf("\n");
  return 0;
}
