# F1TENTH Combined Navigation — Wall Following + Reactive Gap Following + AEB

A single ROS node for the F1TENTH autonomous racing platform that combines
three techniques into one priority-arbitrated controller:

1. **Automatic Emergency Braking (AEB)** — highest priority, TTC-based
2. **Reactive Follow-the-Gap** — obstacle avoidance
3. **PID Wall-Following** — default driving mode

Built from F1TENTH's Lab 3 (Wall Following) and Lab 4 (Follow the Gap) as
independent practice, then extended with a TTC-based safety layer and
combined into a single arbitrated node rather than kept as separate
deliverables — the interesting part of this project is the arbitration
logic between the three, not any one algorithm in isolation.

## How it works

### 1. PID wall following
Two LiDAR scans (one directly left, one offset by a fixed angle) are used
to compute the car's distance and orientation angle relative to the left
wall via trigonometry, then projected forward by a lookahead distance to
account for the car's non-instantaneous response at speed. A PID
controller drives the steering angle to hold a target distance from the
wall; speed is stepped down as steering angle increases.

### 2. Reactive follow-the-gap
Each scan is smoothed and clipped, the closest obstacle point gets a
safety "bubble" zeroed out around it, the widest remaining gap of free
space is found, and the car steers toward the farthest point within that
gap. This is the fallback whenever wall-following's forward-clearance
check detects something too close ahead — including corners the
wall-follow layer alone wouldn't see coming in time.

### 3. Automatic emergency braking
Every scan, the node computes time-to-collision (TTC) per LiDAR beam using
current forward velocity from odometry: `TTC = effective_range / closing_speed`.
If the minimum TTC across all beams drops below a threshold, the car
brakes to a stop immediately — this check runs first and, if triggered,
preempts both of the other two behaviors for that scan.

### Arbitration order
```
AEB triggered?  ──yes──> brake, publish /brake + /brake_bool, skip rest of scan
      │no
      ▼
front clearance ok?  ──no──> follow-the-gap steers this scan
      │yes
      ▼
PID wall-following drives this scan
```

## Package structure
```
combined_nav/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── combined_nav.launch
└── scripts/
    ├── wall_follow.py           # PID wall-following algorithm
    ├── reactive_gap_follow.py   # follow-the-gap algorithm
    └── combined_nav.py          # ties all three together with arbitration
```

## Dependencies
- ROS Noetic
- [f1tenth_simulator](https://github.com/f1tenth/f1tenth_simulator) (or the real F1TENTH car stack)
- `ackermann_msgs`

## Running it
```bash
roslaunch f1tenth_simulator simulator.launch
roslaunch combined_nav combined_nav.launch
```

## Key tunable parameters
| Parameter | File | Purpose |
|---|---|---|
| `kp`, `ki`, `kd` | `wall_follow.py` | PID gains for wall-following |
| `DESIRED_DISTANCE_LEFT` | `wall_follow.py` | target distance from left wall (m) |
| `MAX_STEERING_ANGLE` | `wall_follow.py` | hard steering limit, match your car's real limit |
| `BUBBLE_RADIUS` | `reactive_gap_follow.py` | safety margin around the closest obstacle |
| `FRONT_DANGER_DIST`, `FRONT_CONE_DEG` | `combined_nav.py` | when control hands off to gap-follow |
| `~ttc_threshold` | `combined_nav.py` (ROS param) | seconds of TTC that triggers AEB |
| `~front_offset` | `combined_nav.py` (ROS param) | LiDAR-to-bumper offset used in TTC |

