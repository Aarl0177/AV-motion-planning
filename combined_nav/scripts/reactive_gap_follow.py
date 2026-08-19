#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

BUBBLE_RADIUS = 0.25   # meters, safety bubble around the closest point
MAX_RANGE = 3.0         # meters, clip anything farther than this
WINDOW_SIZE = 5         # samples, moving-average smoothing window
STEER_LIMIT_DEG = 30.0  # cap steering angle for stability


class reactive_follow_gap:
    def __init__(self):
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan,
                                           self.lidar_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped,
                                          queue_size=1)
        self.angle_increment = None

    def preprocess_lidar(self, ranges):
        """
        1. Smooth with a moving-average window to reduce sensor noise.
        2. Clip invalid/far readings to MAX_RANGE so one stray long reading
           doesn't dominate the gap search.
        """
        arr = np.array(ranges, dtype=np.float64)
        arr = np.nan_to_num(arr, nan=0.0, posinf=MAX_RANGE, neginf=0.0)
        arr = np.clip(arr, 0.0, MAX_RANGE)

        kernel = np.ones(WINDOW_SIZE) / WINDOW_SIZE
        smoothed = np.convolve(arr, kernel, mode='same')
        return smoothed

    def find_max_gap(self, free_space_ranges):
        """ Longest run of non-zero entries. Returns (start_i, end_i). """
        masked = np.ma.masked_where(free_space_ranges == 0.0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked)

        if not slices:
            return 0, len(free_space_ranges) - 1
        if isinstance(slices, slice):
            slices = [slices]

        best = max(slices, key=lambda s: s.stop - s.start)
        return best.start, best.stop - 1

    def find_best_point(self, start_i, end_i, ranges):
        """
        Best point in the gap: midpoint of the widest run of the farthest
        readings in that gap -- steadier than taking a single furthest point,
        which can be a noisy outlier.
        """
        segment = ranges[start_i:end_i + 1]
        if len(segment) == 0:
            return (start_i + end_i) // 2

        max_val = np.max(segment)
        farthest_idx = np.where(segment == max_val)[0]
        best_local = int(np.mean(farthest_idx))
        return start_i + best_local

    def lidar_callback(self, data):
        if self.angle_increment is None:
            self.angle_increment = data.angle_increment

        proc_ranges = self.preprocess_lidar(data.ranges)

        closest_idx = int(np.argmin(proc_ranges))
        closest_range = proc_ranges[closest_idx]

        if closest_range > 0:
            bubble_pts = int(BUBBLE_RADIUS / closest_range / data.angle_increment)
        else:
            bubble_pts = 1
        bubble_pts = max(bubble_pts, 1)

        lo = max(0, closest_idx - bubble_pts)
        hi = min(len(proc_ranges), closest_idx + bubble_pts + 1)
        proc_ranges[lo:hi] = 0.0

        start_i, end_i = self.find_max_gap(proc_ranges)
        best_idx = self.find_best_point(start_i, end_i, proc_ranges)

        angle = data.angle_min + best_idx * data.angle_increment
        limit = math.radians(STEER_LIMIT_DEG)
        angle = max(-limit, min(limit, angle))

        angle_deg = abs(math.degrees(angle))
        if angle_deg <= 10.0:
            speed = 1.5
        elif angle_deg <= 20.0:
            speed = 1.0
        else:
            speed = 0.5

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
