# F1TENTH Pure Pursuit — Trajectory Tracking (sim-only adaptation)

An implementation of the Pure Pursuit path-tracking algorithm from
F1TENTH's Lab 6, adapted to run entirely in simulation without
Cartographer/particle-filter localization.

## Why "sim-only adaptation"

The official lab localizes with a particle filter running on a
Cartographer-built SLAM map of the physical car's environment. Working
purely in simulation, that stack isn't available, so this project
substitutes the simulator's ground-truth `/odom` for the pose source, and
records the reference path by letting an existing autonomous driver
([combined_nav](../combined_nav)) complete a lap while a logger node
captures waypoints — rather than mapping with Cartographer and manually
driving a lap for the particle filter to localize against.

## How it works

1. **`waypoint_logger.py`** subscribes to `/odom` and writes an `(x, y)`
   row to a CSV every time the car moves past a minimum spacing threshold,
   producing a reference path CSV from any lap of the track.
2. **`pure_pursuit.py`** loads that CSV, and every odometry update:
   - Searches forward through the waypoint list for a target point at the
     current lookahead distance
   - **Checks LiDAR line-of-sight to that target** — if a wall is actually
     in the way (e.g. the straight-line chord to a far-ahead target cuts
     across the inside of a sharp corner), the lookahead is shrunk in
     steps and re-searched until it finds a target it can actually see a
     clear path to. This is what prevents pure pursuit's classic
     corner-cutting collision failure mode.
   - Transforms the chosen target into the car's local frame and computes
     curvature via `γ = 2y / L²`, converting to a steering angle through
     the bicycle model: `δ = atan(wheelbase · γ)`
   - Slows down proportionally to steering angle magnitude
3. Once the car passes the last recorded waypoint, the path search returns
   nothing further ahead (it doesn't loop back to the start), and the node
   latches into a braking state rather than driving indefinitely.

## Package structure
```
pure_pursuit/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── waypoint_logger.launch
│   └── pure_pursuit.launch
└── scripts/
    ├── waypoint_logger.py
    └── pure_pursuit.py
```

## Dependencies
- ROS Noetic
- [f1tenth_simulator](https://github.com/f1tenth/f1tenth_simulator)
- `ackermann_msgs`

## Running it
```bash
roslaunch f1tenth_simulator simulator.launch

# generate a reference path (any autonomous or manual lap works)
roslaunch pure_pursuit waypoint_logger.launch
# ...let it drive/be driven a lap, then Ctrl-C the logger...

roslaunch pure_pursuit pure_pursuit.launch
```

Add a `MarkerArray` display on `/pure_pursuit/markers` in RViz to see the
logged path (blue dots) and the live lookahead target (red sphere).

## Key tunable parameters
| Parameter | Purpose |
|---|---|
| `lookahead_distance` (launch param → `MAX_LOOKAHEAD`) | ideal lookahead on open track |
| `min_lookahead` (launch param → `MIN_LOOKAHEAD`) | floor the corner-avoidance shrink won't go below |
| `SIGHT_MARGIN` | safety buffer added past the target when checking LiDAR clearance |
| `velocity` | base driving speed |

## Demo
[YouTube link here]

## Credits
Built on the lab handout for **F1TENTH Autonomous Racing, Lab 6 (Pure
Pursuit)**, developed by the Safe Autonomous Systems Lab at the University
of Pennsylvania (Dr. Rahul Mangharam), licensed under
[CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/).
Course materials: [f1tenth.org](http://f1tenth.org/). The sim-only pose
substitution, LiDAR-checked adaptive lookahead, and end-of-path braking
behavior are original additions built on top of the lab's core algorithm.
