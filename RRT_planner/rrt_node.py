#!/usr/bin/env python
from __future__ import print_function
import os
import sys
import csv
import math
import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# catkin's devel-space launcher for THIS file is a stub that exec()s the real
# source rather than running it directly, which means Python's default
# import search (starting from the stub's own directory) can find broken
# stub copies of occupancy_grid.py/rrt.py instead of the real modules. Force
# the real source directory to the front of sys.path so the import below
# always finds the actual code, regardless of how this script got launched.
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

from occupancy_grid import OccupancyGrid
from rrt import RRT

WHEELBASE = 0.50


class RRTNode(object):
    def __init__(self):
        waypoints_file = rospy.get_param("~waypoints_file", "")
        self.goal_lookahead = rospy.get_param("~goal_lookahead", 3.0)
        self.velocity = rospy.get_param("~velocity", 1.2)

        grid_width = rospy.get_param("~grid_width", 6.0)
        grid_height = rospy.get_param("~grid_height", 6.0)
        resolution = rospy.get_param("~grid_resolution", 0.1)
        inflate_radius = rospy.get_param("~inflate_radius", 0.25)
        max_iter = rospy.get_param("~max_iter", 300)
        step_size = rospy.get_param("~step_size", 0.5)
        goal_bias = rospy.get_param("~goal_bias", 0.15)
        goal_threshold = rospy.get_param("~goal_threshold", 0.4)

        self.grid = OccupancyGrid(grid_width, grid_height, resolution, inflate_radius)
        self.rrt = RRT(self.grid, max_iter=max_iter, step_size=step_size,
                        goal_bias=goal_bias, goal_threshold=goal_threshold)

        self.waypoints = self._load_waypoints(waypoints_file)
        self.pose = None
        self.last_idx = 0

        self.drive_pub = rospy.Publisher('/nav', AckermannDriveStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('/rrt/markers', MarkerArray, queue_size=1)

        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)

        rospy.loginfo("RRT node up. %d waypoints loaded%s.",
                       len(self.waypoints),
                       " (goal defaults straight ahead)" if not self.waypoints else "")

    def _load_waypoints(self, path):
        if not path:
            return []
        try:
            pts = []
            with open(path, 'r') as f:
                for row in csv.reader(f):
                    if not row or row[0].startswith('#'):
                        continue
                    pts.append((float(row[0]), float(row[1])))
            return pts
        except IOError:
            rospy.logwarn("Couldn't open waypoints file %s -- planning "
                           "straight ahead instead. Run pure_pursuit's "
                           "waypoint_logger.py first for a real reference path.",
                           path)
            return []

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose = (x, y, yaw)

    def _car_to_world(self, cx, cy):
        x, y, yaw = self.pose
        wx = x + math.cos(yaw) * cx - math.sin(yaw) * cy
        wy = y + math.sin(yaw) * cx + math.cos(yaw) * cy
        return wx, wy

    def _world_goal_in_car_frame(self):
        """ Pick a waypoint goal_lookahead meters ahead along the global path
        (same forward-search idea as pure_pursuit.py) and express it in the
        car's local frame, since the grid/RRT both operate locally. Falls
        back to 'straight ahead' if no waypoint file was loaded. """
        if not self.waypoints:
            return self.goal_lookahead, 0.0

        x, y, yaw = self.pose
        n = len(self.waypoints)
        idx = self.last_idx
        for i in range(n):
            wx, wy = self.waypoints[(idx + i) % n]
            d = math.hypot(wx - x, wy - y)
            if d >= self.goal_lookahead:
                self.last_idx = (idx + i) % n
                dx, dy = wx - x, wy - y
                car_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
                car_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
                return car_x, car_y
        return self.goal_lookahead, 0.0

    def scan_callback(self, data):
        if self.pose is None:
            return

        self.grid.update_from_scan(data.ranges, data.angle_min,
                                    data.angle_increment, data.range_max)

        goal = self._world_goal_in_car_frame()
        start = (0.0, 0.0)  # the car is always the origin of its own local frame

        path, tree = self.rrt.plan(start, goal)
        self._publish_markers(tree, path, goal)

        if not path or len(path) < 2:
            self._publish_drive(0.0, 0.0)  # no path found -- brake, don't guess
            return

        target_x, target_y = path[1]  # first step beyond the car's own position
        self._drive_toward(target_x, target_y)

    def _drive_toward(self, car_x, car_y):
        car_x = max(car_x, 0.05)
        L = math.hypot(car_x, car_y)
        gamma = 2.0 * car_y / (L ** 2) if L > 1e-3 else 0.0
        steering_angle = math.atan(WHEELBASE * gamma)
        steering_angle = max(-math.radians(24), min(math.radians(24), steering_angle))

        speed = self.velocity
        if abs(math.degrees(steering_angle)) > 15:
            speed *= 0.5

        self._publish_drive(steering_angle, speed)

    def _publish_drive(self, steering_angle, speed):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "base_link"
        cmd.drive.steering_angle = steering_angle
        cmd.drive.speed = speed
        self.drive_pub.publish(cmd)

    def _publish_markers(self, tree, path, goal):
        # Everything below is computed in the car's local frame; convert back
        # to world/map coordinates using the current pose so RViz can show it
        # without needing a base_link->map TF to already be set up right.
        arr = MarkerArray()

        tree_marker = Marker()
        tree_marker.header.frame_id = "map"
        tree_marker.ns = "rrt_tree"
        tree_marker.id = 0
        tree_marker.type = Marker.LINE_LIST
        tree_marker.action = Marker.ADD
        tree_marker.scale.x = 0.01
        tree_marker.color.r = tree_marker.color.g = tree_marker.color.b = 0.6
        tree_marker.color.a = 0.6
        for node in tree:
            if node.parent is not None:
                px, py = self._car_to_world(node.parent.x, node.parent.y)
                nx, ny = self._car_to_world(node.x, node.y)
                tree_marker.points.append(Point(x=px, y=py, z=0.0))
                tree_marker.points.append(Point(x=nx, y=ny, z=0.0))
        arr.markers.append(tree_marker)

        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.ns = "rrt_path"
        path_marker.id = 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.05
        path_marker.color.g = 1.0
        path_marker.color.a = 1.0
        for (x, y) in path:
            wx, wy = self._car_to_world(x, y)
            path_marker.points.append(Point(x=wx, y=wy, z=0.0))
        arr.markers.append(path_marker)

        goal_marker = Marker()
        goal_marker.header.frame_id = "map"
        goal_marker.ns = "rrt_goal"
        goal_marker.id = 2
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        gx, gy = self._car_to_world(goal[0], goal[1])
        goal_marker.pose.position.x = gx
        goal_marker.pose.position.y = gy
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.3
        goal_marker.color.r = 1.0
        goal_marker.color.a = 1.0
        arr.markers.append(goal_marker)

        self.marker_pub.publish(arr)


def main(args):
    rospy.init_node("rrt_node", anonymous=True)
    RRTNode()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)