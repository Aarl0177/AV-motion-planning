// plot_results -- C++ version of plot_results() from the NMPC script.
//
//   plot_results <prefix>
//
// reads  <prefix>_log.csv     (t, x, y, psi, v, a, delta, v_target, o0x, o0y, ...)
//        <prefix>_paths.csv   (plan_id, t, reason, x, y)  -- optional, the RRT plans
// writes <prefix>_controls.svg        a(t) and delta(t) with their limits
//        <prefix>_vehicle_states.svg  v(t) vs target and psi(t)
//        <prefix>_clearance.svg       distance to each obstacle vs the safety radius
//        <prefix>_xy_trajectory.svg   trajectory, RRT plans, obstacles at closest approach
//
// Works for both the standalone simulator and a Gazebo run (log_prefix parameter).
// Open the SVGs in any browser; hover a line or marker for its tooltip.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "svg_plot.hpp"

namespace
{

constexpr double kObsRadius = 0.30;
constexpr double kSafeRadius = 0.30 + 0.20 + 0.40;
constexpr double kContact = 0.30 + 0.20;
constexpr double kLaneY = 2.0;
constexpr double kAccelLimit = 2.0;
constexpr double kSteerLimit = M_PI / 10.0;

// Column-oriented CSV table with a header row.
struct Table
{
  std::map<std::string, std::vector<double>> col;
  size_t rows{0};
  bool has(const std::string & name) const {return col.count(name) > 0;}
  const std::vector<double> & operator[](const std::string & name) const {return col.at(name);}
};

std::vector<std::string> split(const std::string & line)
{
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    out.push_back(item);
  }
  return out;
}

bool readLog(const std::string & path, Table & t)
{
  std::ifstream f(path);
  std::string line;
  if (!f || !std::getline(f, line)) {
    return false;
  }
  const auto names = split(line);
  while (std::getline(f, line)) {
    const auto v = split(line);
    if (v.size() != names.size()) {
      continue;  // skip a partially written last line
    }
    for (size_t i = 0; i < v.size(); ++i) {
      t.col[names[i]].push_back(std::stod(v[i]));
    }
    ++t.rows;
  }
  return t.rows > 0;
}

struct Plan
{
  double t{0};
  std::string reason;
  std::vector<double> x, y;
};

std::vector<Plan> readPaths(const std::string & path)
{
  std::map<int, Plan> plans;
  std::ifstream f(path);
  std::string line;
  if (!f || !std::getline(f, line)) {
    return {};
  }
  while (std::getline(f, line)) {
    const auto v = split(line);
    if (v.size() < 5) {
      continue;
    }
    // plan_id, t, reason..., x, y   (x, y are always the last two fields)
    Plan & p = plans[std::stoi(v[0])];
    p.t = std::stod(v[1]);
    p.reason = v[2];
    p.x.push_back(std::stod(v[v.size() - 2]));
    p.y.push_back(std::stod(v[v.size() - 1]));
  }
  std::vector<Plan> out;
  for (auto & kv : plans) {
    out.push_back(std::move(kv.second));
  }
  return out;
}

std::string fmt(double v, int decimals) {return svgplot::fmt(v, decimals);}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: plot_results <prefix>   (reads <prefix>_log.csv)\n");
    return 1;
  }
  const std::string prefix = argv[1];
  Table log;
  if (!readLog(prefix + "_log.csv", log)) {
    std::fprintf(stderr, "could not read %s_log.csv\n", prefix.c_str());
    return 1;
  }
  const std::vector<Plan> plans = readPaths(prefix + "_paths.csv");

  int num_obs = 0;
  while (log.has("o" + std::to_string(num_obs) + "x")) {
    ++num_obs;
  }

  const auto & t = log["t"];
  const auto & x = log["x"];
  const auto & y = log["y"];
  using svgplot::Style;
  using svgplot::kBlue;
  using svgplot::kOrange;
  using svgplot::kInkMuted;
  using svgplot::kInkSecondary;

  // ---------------- controls ----------------
  {
    svgplot::Figure fig(1000, 620);
    auto & a = fig.addAxes();
    a.title = "PID control input: acceleration";
    a.ylabel = "a [m/s²]";
    a.addHLine(kAccelLimit, {kInkMuted, 1.5, "6 4"}, "a_max");
    a.addHLine(-kAccelLimit, {kInkMuted, 1.5, "6 4"}, "a_min");
    a.addLine(t, log["a"], {kBlue}, "a", "acceleration a [m/s²]");

    auto & d = fig.addAxes();
    d.title = "PID control input: steering angle";
    d.ylabel = "δ [rad]";
    d.xlabel = "Time [s]";
    d.addHLine(kSteerLimit, {kInkMuted, 1.5, "6 4"}, "+π/10");
    d.addHLine(-kSteerLimit, {kInkMuted, 1.5, "6 4"}, "−π/10");
    d.addLine(t, log["delta"], {kBlue}, "δ", "steering angle δ [rad]");
    fig.save(prefix + "_controls.svg");
  }

  // ---------------- vehicle states ----------------
  {
    svgplot::Figure fig(1000, 620);
    auto & v = fig.addAxes();
    v.title = "Vehicle speed";
    v.ylabel = "v [m/s]";
    double v_peak = 0.5;
    for (double vv : log["v"]) {v_peak = std::max(v_peak, vv);}
    v.ymin = 0.0;
    v.ymax = 1.4 * v_peak;  // head-room for the legend
    v.addLine(t, log["v"], {kBlue}, "v");
    if (log.has("v_target")) {
      v.addLine(t, log["v_target"], {kOrange, 2.0, "6 4"}, "v target");
    }
    auto & p = fig.addAxes();
    p.title = "Yaw angle";
    p.ylabel = "ψ [rad]";
    p.xlabel = "Time [s]";
    p.addLine(t, log["psi"], {kBlue}, "ψ");
    fig.save(prefix + "_vehicle_states.svg");
  }

  // ---------------- clearance + closest approach ----------------
  std::vector<size_t> closest(num_obs, 0);
  std::vector<double> min_d(num_obs, 1e9);
  {
    svgplot::Figure fig(1000, 440);
    auto & c = fig.addAxes();
    c.title = "Distance from the car to each obstacle (centre to centre)";
    c.ylabel = "distance [m]";
    c.xlabel = "Time [s]";
    c.ymin = 0.0;
    c.ymax = 4.0;  // zoom on the region that matters
    c.addHLine(kSafeRadius, {kOrange, 1.5, "6 4"}, "safe radius " + fmt(kSafeRadius, 1) + " m");
    c.addHLine(kContact, {kInkMuted, 1.5, "2 4"}, "contact " + fmt(kContact, 1) + " m");
    for (int j = 0; j < num_obs; ++j) {
      const auto & ox = log["o" + std::to_string(j) + "x"];
      const auto & oy = log["o" + std::to_string(j) + "y"];
      std::vector<double> d(log.rows);
      for (size_t i = 0; i < log.rows; ++i) {
        d[i] = std::hypot(x[i] - ox[i], y[i] - oy[i]);
        if (d[i] < min_d[j]) {
          min_d[j] = d[i];
          closest[j] = i;
        }
      }
      const std::string name = "Obstacle " + std::to_string(j + 1);
      // One hue for all obstacles; each line is named by a direct label at its minimum.
      c.addLine(t, d, {kBlue}, "", name + ": min " + fmt(min_d[j], 2) + " m at t = " +
        fmt(t[closest[j]], 1) + " s");
      c.addMarker({t[closest[j]], min_d[j], kBlue, 'o', "", name + " closest approach"});
      c.addText({t[closest[j]], min_d[j], name + " · " + fmt(min_d[j], 2) + " m", "middle", 0, 18,
          kInkSecondary});
    }
    fig.save(prefix + "_clearance.svg");
  }

  // ---------------- x-y trajectory ----------------
  {
    svgplot::Figure fig(1200, 520);
    auto & ax = fig.addAxes();
    ax.title = "Vehicle trajectory, RRT plans and obstacles at closest approach";
    ax.xlabel = "x position [m]";
    ax.ylabel = "y position [m]";
    ax.equal_aspect = true;
    ax.legend_bottom = true;

    ax.addHLine(kLaneY, {kInkMuted, 1.5, "8 6"}, "reference lane y = " + fmt(kLaneY, 1), true);
    for (size_t k = 0; k < plans.size(); ++k) {
      ax.addLine(plans[k].x, plans[k].y, {kInkMuted, 1.2, "", 0.6}, k == 0 ? "RRT plans" : "",
        "RRT plan " + std::to_string(k) + " at t = " + fmt(plans[k].t, 1) + " s (" +
        plans[k].reason + ")");
    }
    for (int j = 0; j < num_obs; ++j) {
      const auto & ox = log["o" + std::to_string(j) + "x"];
      const auto & oy = log["o" + std::to_string(j) + "y"];
      const size_t i = closest[j];
      ax.addLine(ox, oy, {kOrange, 2.0, "6 4"}, j == 0 ? "obstacle paths" : "",
        "Obstacle " + std::to_string(j + 1) + " path");
      ax.addCircle(ox[i], oy[i], kObsRadius, {kOrange, 2.0}, j == 0 ? "obstacle at closest approach" : "");
      ax.addCircle(ox[i], oy[i], kSafeRadius, {kOrange, 1.2, "2 4"}, j == 0 ? "safety radius" : "");
      ax.addLine({x[i], ox[i]}, {y[i], oy[i]}, {kInkMuted, 1.0}, "",
        "closest approach " + fmt(min_d[j], 2) + " m");
      // label on the side of the start point away from the lane line
      const double dy = oy.front() > kLaneY ? -10 : 20;
      ax.addText({ox.front(), oy.front(), "Obs " + std::to_string(j + 1), "middle", 0, dy});
    }
    ax.addLine(x, y, {kBlue, 2.5}, "vehicle trajectory");
    ax.addMarker({x.front(), y.front(), kBlue, 'o', "start",
        "start (" + fmt(x.front(), 2) + ", " + fmt(y.front(), 2) + ")"});
    ax.addMarker({x.back(), y.back(), kBlue, 'x', "end",
        "end (" + fmt(x.back(), 2) + ", " + fmt(y.back(), 2) + ") at t = " + fmt(t.back(), 1) + " s"});
    fig.save(prefix + "_xy_trajectory.svg");
  }

  std::printf("Wrote %s_{controls,vehicle_states,clearance,xy_trajectory}.svg\n", prefix.c_str());
  std::printf("Run time %.1f s, %zu RRT plans. Closest approach (safe radius %.2f m):",
    t.back(), plans.size(), kSafeRadius);
  for (int j = 0; j < num_obs; ++j) {
    std::printf("  obstacle %d: %.2f m", j + 1, min_d[j]);
  }
  std::printf("\n");
  return 0;
}
