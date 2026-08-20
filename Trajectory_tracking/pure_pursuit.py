#!/usr/bin/env python
from __future__ import print_function
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

WHEELBASE = 0.50            # meters, matches CAR_LENGTH used elsewhere
MAX_LOOKAHEAD = 1.0         # meters -- the "ideal" lookahead on open track
MIN_LOOKAHEAD = 0.5         # meters -- floor; never shrink below this
LOOKAHEAD_STEP = 0.1        # meters -- how much to shrink per retry
SIGHT_MARGIN = 0.15         # meters -- safety buffer past the target point
VELOCITY = 1.5              # base m/s


class PurePursuit(object):
    def __init__(self):
        waypoints_file = rospy.get_param("~waypoints_file")
        self.max_lookahead = rospy.get_param("~lookahead_distance", MAX_LOOKAHEAD)
        self.min_lookahead = rospy.get_param("~min_lookahead", MIN_LOOKAHEAD)
        self.velocity = rospy.get_param("~velocity", VELOCITY)

        self.waypoints = self._load_waypoints(waypoints_file)
        if len(self.waypoints) < 2:
            rospy.logfatal("Need >=2 waypoints in %s -- drive a lap with "
                            "waypoint_logger.py first.", waypoints_file)

        self.pose = None       # (x, y, yaw) in the map/world frame
        self.last_scan = None
        self.last_idx = 0
        self.finished = False

        self.drive_pub = rospy.Publisher('/nav', AckermannDriveStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('/pure_pursuit/markers', MarkerArray, queue_size=1)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)

        rospy.Timer(rospy.Duration(1.0), self._publish_waypoint_markers)  # periodic, cheap

    def scan_callback(self, msg):
        self.last_scan = msg

    def _load_waypoints(self, path):
        pts = []
        with open(path, 'r') as f:
            for row in csv.reader(f):
                if not row or row[0].startswith('#'):
                    continue
                pts.append((float(row[0]), float(row[1])))
        return pts

    def _publish_waypoint_markers(self, _event=None):
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = "map"
        m.ns = "waypoints"
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.scale.x = 0.08
        m.scale.y = 0.08
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.6, 1.0, 1.0
        m.points = [Point(x=x, y=y, z=0.0) for (x, y) in self.waypoints]
        arr.markers.append(m)
        self.marker_pub.publish(arr)

    def _publish_target_marker(self, x, y):
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = "map"
        m.ns = "target"
        m.id = 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.color.r, m.color.a = 1.0, 1.0
        arr.markers.append(m)
        self.marker_pub.publish(arr)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose = (x, y, yaw)
        self._track()

    def _range_at_angle(self, scan, angle_rad):
        """ Same idea as wall_follow.py's getRange: look up the measured
        LiDAR distance at a given angle (0 = forward, +90deg = left). """
        index = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
        index = max(0, min(len(scan.ranges) - 1, index))
        r = scan.ranges[index]
        if math.isnan(r) or math.isinf(r) or r <= 0.0:
            return scan.range_max
        return r

    def _target_is_visible(self, car_x, car_y):
        """ Is there actually clear space, per the current scan, all the way
        out to this candidate target? If the measured range along that
        bearing is shorter than the distance to the target, the target is
        sitting behind (or inside) a wall -- e.g. a chord cutting across the
        inside of a corner -- and shouldn't be used. """
        if self.last_scan is None:
            return True  # no scan yet; don't block on startup
        angle = math.atan2(car_y, car_x)
        dist_to_target = math.hypot(car_x, car_y)
        measured = self._range_at_angle(self.last_scan, angle)
        return measured > dist_to_target + SIGHT_MARGIN

    def _find_target(self):
        """ Search forward from the last matched index for the first waypoint
        at or beyond the given lookahead. Does NOT wrap around -- once the
        remaining path is shorter than the lookahead, returns None, which
        signals 'nothing left ahead' rather than looping back to index 0. """
        x, y, _ = self.pose
        n = len(self.waypoints)
        for i in range(self.last_idx, n):
            wx, wy = self.waypoints[i]
            d = math.hypot(wx - x, wy - y)
            if d >= self.current_lookahead:
                return wx, wy, i
        return None

    def _find_visible_target(self):
        """ Try the full lookahead first; if the straight line to that target
        isn't actually clear per the LiDAR, shrink the lookahead and retry
        down to MIN_LOOKAHEAD. This is what prevents the pure-pursuit arc
        from cutting across the inside of a sharp corner into a wall. """
        x, y, yaw = self.pose
        lookahead = self.max_lookahead

        while lookahead >= self.min_lookahead:
            self.current_lookahead = lookahead
            result = self._find_target()
            if result is not None:
                wx, wy, wi = result
                dx, dy = wx - x, wy - y
                car_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
                car_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
                if self._target_is_visible(car_x, car_y):
                    self.last_idx = wi
                    return wx, wy
            lookahead -= LOOKAHEAD_STEP

        # nothing visible even at the minimum lookahead -- try one more
        # uncontested search at min_lookahead; if that also comes up empty,
        # there's genuinely no path left ahead of us.
        self.current_lookahead = self.min_lookahead
        result = self._find_target()
        if result is not None:
            wx, wy, wi = result
            self.last_idx = wi
            return wx, wy
        return None  # end of the recorded path

    def _publish_drive(self, steering_angle, speed):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "base_link"
        cmd.drive.steering_angle = steering_angle
        cmd.drive.speed = speed
        self.drive_pub.publish(cmd)

    def _track(self):
        if not self.waypoints or self.pose is None:
            return

        if self.finished:
            self._publish_drive(0.0, 0.0)  # keep re-publishing zero, don't just go silent
            return

        target = self._find_visible_target()
        if target is None:
            self.finished = True
            rospy.loginfo("Reached the end of the recorded path -- stopping.")
            self._publish_drive(0.0, 0.0)
            return

        tx, ty = target
        self._publish_target_marker(tx, ty)

        x, y, yaw = self.pose
        dx, dy = tx - x, ty - y
        # rotate the target into the car's local frame (+x forward, +y left)
        car_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        car_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        car_x = max(car_x, 0.05)  # guard against a near-zero/behind-us divide

        L = math.hypot(car_x, car_y)
        # gamma = 2|y| / L^2 per the handout; we keep the SIGN of car_y (not
        # the absolute value) here because the sign is what tells the
        # steering command which way to turn.
        gamma = 2.0 * car_y / (L ** 2) if L > 1e-3 else 0.0
        steering_angle = math.atan(WHEELBASE * gamma)
        steering_angle = max(-math.radians(24), min(math.radians(24), steering_angle))

        speed = self.velocity
        angle_deg = abs(math.degrees(steering_angle))
        if angle_deg > 15:
            speed *= 0.5
        elif angle_deg > 8:
            speed *= 0.75

        self._publish_drive(steering_angle, speed)


def main(args):
    rospy.init_node("pure_pursuit_node", anonymous=True)
    PurePursuit()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)