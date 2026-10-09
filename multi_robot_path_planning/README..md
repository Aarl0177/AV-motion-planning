#Multi-Robot A* + NMPC with V2V Communication

Decentralized motion planning for three cars sharing a map with static pillars and a moving pedestrian. Each car runs the same stack:

Global planner (A*): plans a collision-free route on an inflated occupancy grid. The route is then shortcut-pruned, Chaikin-smoothed and resampled into a reference path.
V2V communication: every step, each car broadcasts its state [x, y, ψ, v] and its NMPC predicted trajectory. The other cars receive it one step later.
Local planner (NMPC): built with CasADi and IPOPT on a kinematic bicycle model. Each car tracks its own A* path in the path frame and avoids everything else with soft distance constraints:
Other cars: predicted from their shared plans.
The pedestrian: sensed rather than communicated, and predicted with constant velocity.
The pillars: treated as static obstacles.
