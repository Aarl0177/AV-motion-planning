#!/usr/bin/env python
from __future__ import print_function
import sys
import csv
import math
import rospy
from nav_msgs.msg import Odometry


class WaypointLogger(object):
    """
    Drive a lap manually (keyboard/joystick) with this node running, then
    Ctrl-C it -- it writes one (x, y) row per position sample, spaced by
    min_spacing, to output_file. That CSV becomes the reference path for
    pure_pursuit.py and the long-range goal source for the RRT planner.

    NOTE: the official lab logs from /pf/pose/odom (the particle filter's
    estimate). We don't have Cartographer/AMCL running in the simulator, so
    this logs straight from /odom (simulator ground truth) instead -- fine
    for sim practice, but swap the topic back to /pf/pose/odom if you move
    to the real car with localization running.
    """

    def __init__(self):
        out_path = rospy.get_param("~output_file")
        self.min_dist = rospy.get_param("~min_spacing", 0.15)  # meters
        self.last_xy = None

        self.f = open(out_path, 'w')
        self.writer = csv.writer(self.f)
        self.writer.writerow(["# x", "y"])

        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Logging waypoints to %s -- drive a lap, then Ctrl-C.", out_path)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_xy is not None:
            d = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            if d < self.min_dist:
                return
        self.last_xy = (x, y)
        self.writer.writerow([x, y])
        self.f.flush()

    def shutdown(self):
        self.f.close()
        rospy.loginfo("Waypoint log closed.")


def main(args):
    rospy.init_node("waypoint_logger", anonymous=True)
    WaypointLogger()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
