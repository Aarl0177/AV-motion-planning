# F1TENTH Local Motion Planner (RRT)

This is a local motion planner for a simulated F1TENTH race car. It watches
the LiDAR, notices when something's in the way, and figures out a path
around it in real time.

It works alongside a [pure pursuit](../pure_pursuit) node that follows a
recorded lap around the track. That gives the car something to aim for.
This planner's job is just to make sure it doesn't drive into a wall on
the way there.

## What's actually going on here

Every time a new LiDAR scan comes in, the car:

1. Builds a little grid map of what's around it right now (not a
   permanent map, just "here's what I can see this instant")
2. Picks a goal point a few meters ahead, based on wherever the recorded
   path is heading
3. Tries to find a path from where it is to that goal without hitting
   anything
4. Steers toward the first step of whatever path it found

That's it. It does this over and over, dozens of times a second, so it's
constantly re-planning as the world changes around it.

### How it finds a path

RRT basically throws darts. It picks random points nearby, checks if it
can reach them without hitting a wall, and keeps building a tree of
connected points until one of them is close enough to the goal. It's
messy-looking but it works, and it doesn't need much math.

## Running it

You need the simulator running first, obviously:

```
roslaunch f1tenth_simulator simulator.launch
```

Then run it:

```
roslaunch rrt_planner rrt.launch
```

Open RViz and add a MarkerArray on `/rrt/markers` if you want to actually
see it thinking -- grey lines are everywhere it explored, green is the
path it picked.

## Things I'd still change if I kept going

- It's all Python, and it shows -- re-planning from scratch every single
  scan is not free, and it's nowhere near the ~30Hz the lab's own C++
  version manages. Fine for messing around in sim, not fine for a real
  race.
- No RRT* -- this is the plain version, so it doesn't try to improve a
  path after finding one.
- The goal point it aims for is always the same fixed distance ahead,
  even if that spot happens to be surrounded by obstacles. A smarter
  version would pull the goal in closer when things get crowded.

