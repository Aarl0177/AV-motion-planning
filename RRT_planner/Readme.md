# F1TENTH RRT Planner

This is a project I built to practice local motion planning for a simulated race car (the F1TENTH platform). The car uses RRT (Rapidly-exploring Random Tree) to dodge obstacles in real time using its LiDAR.

## What it actually does

Every time a new LiDAR scan comes in, the car:

1. Builds a small grid map of whatever's around it right now
2. Grows a random tree out from where it's sitting, trying to reach a point further down the track
3. Takes whichever branch of that tree actually got close enough to the goal
4. Steers toward the first step along that path

If it can't find a path in time, it just brakes instead of guessing and driving into something.

I later added A* as a second option so I could actually compare a grid-search planner against the random-sampling one, instead of just reading about the difference. You can switch between the two with one launch argument, no code changes needed.

## Why it's built this way

This is based on F1TENTH's Lab 7 (motion planning). The lab has you implement RRT for local obstacle avoidance, and optionally compare it against grid-based search like A* or Dijkstra's. I did both, since I wanted to see the actual difference for myself.

I'm running everything in simulation, not on a real car, so a few things are simplified from how the lab originally describes them:

- No Cartographer or particle filter — I just use the simulator's ground-truth position directly
- The "global path" it plans toward comes from a lap I drove earlier with a different node I built (wall-following), recorded with a simple waypoint logger

## Running it

```bash
roslaunch f1tenth_simulator simulator.launch
roslaunch rrt_planner rrt.launch
```

For A* instead of RRT:

```bash
roslaunch rrt_planner rrt.launch planner_type:=astar
```

Add a MarkerArray display in RViz on `/rrt/markers` to actually watch it work — grey lines are the tree it explored that cycle, green is the path it picked.

## What's in here

- `occupancy_grid.py` — turns raw LiDAR ranges into a small local grid map
- `rrt.py` — the RRT algorithm itself
- `astar.py` — the A* alternative, built to share the same grid and node structure as RRT so the visualization code didn't need to change
- `rrt_node.py` — the actual ROS node, wires everything to the sensors and the drive commands

## Things worth tuning

- `max_iter` / `step_size` — how many times RRT tries per cycle, and how far each step goes. Biggest knob for speed vs. how good the path is.
- `inflate_radius` — how much safety margin obstacles get. Too small and it clips corners, too big and it can't find a path through tight gaps.
- `goal_lookahead` — how far down the recorded path it aims for.

## Honest limitations

- It's Python, so it's slower than the C++ version the lab expects for real racing speed. Fine for learning, not something I'd race with.
- This is plain RRT, not RRT* — it just finds a path that works, it doesn't try to optimize it.
- The goal point is always a fixed distance ahead, no matter how blocked the space around it currently is. A smarter version would shrink that distance when things get crowded.

## Credit

Built on F1TENTH course materials from the Safe Autonomous Systems Lab at the University of Pennsylvania (f1tenth.org), licensed under CC BY-NC-SA 4.0. The RRT algorithm follows the structure from LaValle & Kuffner's original paper, which the lab points you to.
