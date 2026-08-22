#!/usr/bin/env python
"""
rrt.py
Minimal RRT implementation operating in the car's local (x, y) frame.
Mirrors the standard pseudocode from the handout / LaValle & Kuffner:
sample -> nearest -> steer -> collision-check -> add to tree -> repeat.
"""
import math
import random


class Node(object):
    __slots__ = ("x", "y", "parent")

    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent


class RRT(object):
    def __init__(self, occupancy_grid, max_iter=300, step_size=0.5,
                 goal_bias=0.15, goal_threshold=0.4):
        self.grid = occupancy_grid
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.goal_threshold = goal_threshold

    def sample_free(self, goal):
        """ Goal-biased sampling: mostly sample uniformly across the grid's
        forward/lateral extent; occasionally sample the goal directly so the
        tree gets pulled toward it instead of wandering. """
        if random.random() < self.goal_bias:
            return goal
        x = random.uniform(0.0, self.grid.height_m)
        y = random.uniform(-self.grid.width_m / 2.0, self.grid.width_m / 2.0)
        return (x, y)

    def nearest(self, tree, point):
        best, best_d = None, float("inf")
        for node in tree:
            d = (node.x - point[0]) ** 2 + (node.y - point[1]) ** 2
            if d < best_d:
                best, best_d = node, d
        return best

    def steer(self, nearest_node, sampled_point):
        dx = sampled_point[0] - nearest_node.x
        dy = sampled_point[1] - nearest_node.y
        dist = math.hypot(dx, dy)
        if dist <= self.step_size:
            return sampled_point
        scale = self.step_size / dist
        return (nearest_node.x + dx * scale, nearest_node.y + dy * scale)

    def collision_free(self, p1, p2):
        return self.grid.line_is_free(p1, p2)

    def is_goal(self, node, goal):
        return math.hypot(node.x - goal[0], node.y - goal[1]) <= self.goal_threshold

    def find_path(self, goal_node):
        path = []
        node = goal_node
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        path.reverse()
        return path

    def plan(self, start, goal):
        """ Returns (path, tree): path is a list of (x, y) from start to goal
        (empty if none found within max_iter); tree is every Node explored,
        for visualization. """
        root = Node(start[0], start[1])
        tree = [root]

        for _ in range(self.max_iter):
            sample = self.sample_free(goal)
            nearest_node = self.nearest(tree, sample)
            new_point = self.steer(nearest_node, sample)

            if not self.collision_free((nearest_node.x, nearest_node.y), new_point):
                continue

            new_node = Node(new_point[0], new_point[1], parent=nearest_node)
            tree.append(new_node)

            if self.is_goal(new_node, goal):
                return self.find_path(new_node), tree

        return [], tree  # no path found within max_iter
