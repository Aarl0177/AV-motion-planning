#!/usr/bin/env python
"""
occupancy_grid.py
A small car-frame-local occupancy grid for RRT collision checking.
Grid is axis-aligned in the CAR's frame: +x forward, +y left (REP-103),
origin at the car's current position. Rebuilt from scratch every scan --
this is a local planner, not a persistent map, so we don't need history.
"""
import math
import numpy as np


class OccupancyGrid(object):
    def __init__(self, width_m, height_m, resolution, inflate_radius):
        """
        width_m: total grid width (left-right, y direction), meters
        height_m: total grid extent forward (x direction), meters
        resolution: meters per cell
        inflate_radius: meters to inflate every obstacle cell by (car
                         half-width + safety margin)
        """
        self.resolution = resolution
        self.width_m = width_m
        self.height_m = height_m
        self.inflate_cells = max(1, int(round(inflate_radius / resolution)))

        self.cols = int(round(width_m / resolution))   # y axis (left-right)
        self.rows = int(round(height_m / resolution))   # x axis (forward)
        self.col_offset = self.cols // 2                # y=0 sits mid-grid

        self.grid = np.zeros((self.rows, self.cols), dtype=np.uint8)  # 0=free, 1=occupied

    def world_to_grid(self, x, y):
        row = int(x / self.resolution)
        col = int(y / self.resolution) + self.col_offset
        return row, col

    def in_bounds(self, row, col):
        return 0 <= row < self.rows and 0 <= col < self.cols

    def update_from_scan(self, ranges, angle_min, angle_increment, max_range):
        """ Mark every valid laser hit (plus an inflation margin) as occupied. """
        self.grid.fill(0)
        angle = angle_min
        for r in ranges:
            if math.isfinite(r) and 0.0 < r <= max_range:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                if x >= 0.0:  # only the forward half-plane matters for this grid
                    row, col = self.world_to_grid(x, y)
                    self._mark_occupied(row, col)
            angle += angle_increment

    def _mark_occupied(self, row, col):
        r0 = max(0, row - self.inflate_cells)
        r1 = min(self.rows, row + self.inflate_cells + 1)
        c0 = max(0, col - self.inflate_cells)
        c1 = min(self.cols, col + self.inflate_cells + 1)
        if 0 <= row < self.rows and 0 <= col < self.cols:
            self.grid[r0:r1, c0:c1] = 1

    def line_is_free(self, p1, p2):
        """ Sample along the straight line between two world points and check
        every cell it passes through is free. """
        x0, y0 = p1
        x1, y1 = p2
        row0, col0 = self.world_to_grid(x0, y0)
        row1, col1 = self.world_to_grid(x1, y1)

        steps = max(abs(row1 - row0), abs(col1 - col0), 1)
        for i in range(steps + 1):
            t = i / float(steps)
            row = int(round(row0 + t * (row1 - row0)))
            col = int(round(col0 + t * (col1 - col0)))
            if not self.in_bounds(row, col) or self.grid[row, col] == 1:
                return False
        return True
