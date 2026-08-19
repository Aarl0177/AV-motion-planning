#!/usr/bin/env python
"""
combined_nav.py
Combines Lab 3 (PID wall following) and Lab 4 (reactive follow-the-gap)
into one node with simple safety arbitration:
  - Wall-following drives the car by default.
  - If the closest point in a forward cone drops below FRONT_DANGER_DIST,
    control hands to follow-the-gap for that scan to steer around the
    obstacle, then returns to wall-following once the path is clear again.

Run this instead of wall_follow.py / reactive_gap_follow.py individually.
"""
from __future__ import print_function
import sys
import math
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

from wall_follow import WallFollow, DESIRED_DISTANCE_LEFT, VELOCITY
from reactive_gap_follow import reactive_follow_gap

FRONT_DANGER_DIST = 1.50    # meters -- below this, hand off to gap-follow
FRONT_CONE_DEG = 20.0      # width of the forward danger-check cone


class CombinedNav(object):
    def __init__(self):
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        # This node owns the single subscriber/publisher pair. Create the
        # publisher FIRST, then hand it to each reused sub-object below --
        # both WallFollow and reactive_follow_gap need it before any scan
        # callback fires.
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped,
                                          queue_size=1)

        # Reuse each lab's algorithm methods directly, without letting them
        # create their own subscriber/publisher via __init__.
        self.wall_follower = WallFollow.__new__(WallFollow)
        self.wall_follower.prev_error = 0.0
        self.wall_follower.integral = 0.0
        self.wall_follower.prev_time = None
        self.wall_follower.drive_pub = self.drive_pub

        self.gap_follower = reactive_follow_gap.__new__(reactive_follow_gap)
        self.gap_follower.angle_increment = None
        self.gap_follower.drive_pub = self.drive_pub

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan,
                                           self.lidar_callback, queue_size=1)

    def _front_is_clear(self, data):
        half_cone = math.radians(FRONT_CONE_DEG / 2.0)
        center_idx = int((0.0 - data.angle_min) / data.angle_increment)
        half_span = int(half_cone / data.angle_increment)
        lo = max(0, center_idx - half_span)
        hi = min(len(data.ranges), center_idx + half_span + 1)

        front = [r for r in data.ranges[lo:hi]
                 if not math.isnan(r) and not math.isinf(r)]
        if not front:
            return True
        return min(front) > FRONT_DANGER_DIST

    def lidar_callback(self, data):
        if self._front_is_clear(data):
            error, _ = self.wall_follower.followLeft(data, DESIRED_DISTANCE_LEFT)
            self.wall_follower.pid_control(error, VELOCITY)  # publishes directly
        else:
            self.gap_follower.lidar_callback(data)            # publishes directly


def main(args):
    rospy.init_node("CombinedNav_node", anonymous=True)
    cn = CombinedNav()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)