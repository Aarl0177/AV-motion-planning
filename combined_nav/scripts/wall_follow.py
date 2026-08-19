#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

# ---- PID CONTROL PARAMS (tune these in sim) ----
kp = 1.2
kd = 0.10
ki = 0.0021
servo_offset = 0.0

# ---- WALL FOLLOW PARAMS ----
ANGLE_RANGE = 270              # Hokuyo 10LX scan span (deg)
DESIRED_DISTANCE_LEFT = 0.95    # meters, set point for left-wall following
VELOCITY = 3.0                 # base m/s; actual speed is stepped by steering angle
CAR_LENGTH = 0.50
LOOKAHEAD = 1.0                # L, meters -- how far ahead we project D_t

THETA_DEG = 50.0               # angle between the two laser scans used for alpha
ERROR_CLAMP = 2.0              # meters, guards against alpha/error spikes at open corners
MAX_STEERING_ANGLE = math.radians(24.0)  # match this to your simulator's max_steering_angle param


class WallFollow:
    """ Implements left-wall following on the car using a PID controller. """

    def __init__(self):
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan,
                                           self.lidar_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped,
                                          queue_size=1)

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def getRange(self, data, angle_deg):
        """
        data: LaserScan message
        angle_deg: 0 deg = straight ahead, +90 deg = directly LEFT,
                   -90 deg = directly RIGHT (REP-103 convention).
        Returns range in meters; NaN/inf/zero become range_max so a bad
        reading doesn't blow up the controller.
        """
        angle_rad = math.radians(angle_deg)
        index = int(round((angle_rad - data.angle_min) / data.angle_increment))
        index = max(0, min(len(data.ranges) - 1, index))
        r = data.ranges[index]
        if math.isnan(r) or math.isinf(r) or r <= 0.0:
            return data.range_max
        return r

    def followLeft(self, data, leftDist):
        """
        Returns (error, alpha) for left-wall following.
        b: scan directly left (90 deg). a: scan THETA_DEG toward the front
        of directly-left, i.e. at (90 - THETA_DEG) deg.
        """
        theta = THETA_DEG
        b = self.getRange(data, 90.0)
        a = self.getRange(data, 90.0 - theta)

        theta_rad = math.radians(theta)
        alpha = math.atan2(a * math.cos(theta_rad) - b, a * math.sin(theta_rad))

        D_t = b * math.cos(alpha)
        D_t1 = D_t + LOOKAHEAD * math.sin(alpha)

        error = leftDist - D_t1
        error = max(-ERROR_CLAMP, min(ERROR_CLAMP, error))
        return error, alpha

    def pid_control(self, error, velocity):
        global kp, ki, kd

        now = rospy.Time.now().to_sec()
        dt = (now - self.prev_time) if self.prev_time is not None else 0.05
        dt = max(dt, 1e-3)
        self.prev_time = now

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # error = leftDist - D_t1, so a POSITIVE error means the car is too
        # CLOSE to the left wall. To move away from the wall it needs to
        # steer RIGHT, i.e. a NEGATIVE angle (+90 deg = left in REP-103).
        # Hence the leading minus sign -- an earlier version of this file
        # had this backwards, which is why the car was hugging the wall.
        angle = -(kp * error + ki * self.integral + kd * derivative)
        angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, angle))

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
        return angle, speed

    def lidar_callback(self, data):
        error, alpha = self.followLeft(data, DESIRED_DISTANCE_LEFT)
        self.pid_control(error, VELOCITY)


def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)