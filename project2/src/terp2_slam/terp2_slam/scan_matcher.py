# file: terp2_slam/scan_matcher.py
#!/usr/bin/env python3
import math, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

SEARCH_YAW   = np.deg2rad(5)     # ±5°
SEARCH_XY    = 0.10              # ±10 cm
STEP_YAW     = np.deg2rad(0.5)
STEP_XY      = 0.02

def scan_to_xy(scan):
    angles = scan.angle_min + np.arange(len(scan.ranges))*scan.angle_increment
    ranges = np.asarray(scan.ranges)
    valid  = np.isfinite(ranges)
    xy = np.stack([ranges[valid]*np.cos(angles[valid]),
                   ranges[valid]*np.sin(angles[valid])], axis=1)
    return xy                     # shape (N,2)

def score(a, b):
    """return (# of points within 5 cm) / N"""
    if len(a)==0 or len(b)==0:  return 0.0
    # build simple 2-D grid hash for b
    res  = 0.05
    h = {(int(x/res), int(y/res)) for x,y in b}
    cnt = 0
    for x,y in a:
        if (int(x/res), int(y/res)) in h:
            cnt += 1
    return cnt / len(a)

class ScanMatcher(Node):
    def __init__(self):
        super().__init__("scan_matcher")
        self.prev_xy = None
        self.pose_xy = np.zeros(2)   # running odom in xy
        self.pose_yaw= 0.0

        self.br = TransformBroadcaster(self)
        self.create_subscription(LaserScan, "/scan_filtered", self.cb, 10)

    def cb(self, scan):
        cur_xy = scan_to_xy(scan)
        if self.prev_xy is None:
            self.prev_xy = cur_xy
            return

        best_s, best_dx, best_dy, best_dyaw = -1,0,0,0
        for dyaw in np.arange(-SEARCH_YAW, SEARCH_YAW+1e-3, STEP_YAW):
            c,s = math.cos(dyaw), math.sin(dyaw)
            rot = np.array([[c,-s],[s,c]])
            rot_xy = (rot @ self.prev_xy.T).T
            for dx in np.arange(-SEARCH_XY, SEARCH_XY+1e-3, STEP_XY):
                for dy in np.arange(-SEARCH_XY, SEARCH_XY+1e-3, STEP_XY):
                    trans_xy = rot_xy + np.array([dx,dy])
                    sc = score(cur_xy, trans_xy)
                    if sc > best_s:
                        best_s, best_dx, best_dy, best_dyaw = sc,dx,dy,dyaw

        # apply increment to running pose
        c,s = math.cos(self.pose_yaw), math.sin(self.pose_yaw)
        self.pose_xy += np.array([c*best_dx - s*best_dy,
                                  s*best_dx + c*best_dy])
        self.pose_yaw += best_dyaw

        # publish TF (odom_frontend → base_link)
        t = TransformStamped()
        t.header.stamp    = scan.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id  = "base_link"
        t.transform.translation.x = float(self.pose_xy[0])
        t.transform.translation.y = float(self.pose_xy[1])
        t.transform.translation.z = 0.0
        qz = math.sin(self.pose_yaw/2)
        qw = math.cos(self.pose_yaw/2)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.br.sendTransform(t)

        self.prev_xy = cur_xy

def main():
    rclpy.init()
    rclpy.spin(ScanMatcher())
    rclpy.shutdown()
