#!/usr/bin/env python3
""" 
box_filter.py  –  filter laser scan points that are inside box defined by min/max x,y,z
 Since our LIDAR intersects the robot, we need to filter out points on the robot
"""

import rclpy, math, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener

class BoxFilter(Node):
    def __init__(self):
        super().__init__("box_filter")

        # ------- (ROS parameters defined leveraging Robot Geometry --------
        self.declare_parameter("box_frame",  "lidar_link")
        self.declare_parameter("min_x", -0.30); self.declare_parameter("max_x", 0.30)
        self.declare_parameter("min_y", -0.30); self.declare_parameter("max_y", 0.30)
        self.declare_parameter("min_z", -0.40); self.declare_parameter("max_z", 0.80)
        self.box = np.array([
            self.get_parameter("min_x").value, self.get_parameter("max_x").value,
            self.get_parameter("min_y").value, self.get_parameter("max_y").value,
            self.get_parameter("min_z").value, self.get_parameter("max_z").value,
        ], dtype=np.float32)
        self.box_frame = self.get_parameter("box_frame").value

        # ------- TF & topics --------------------------------------------------
        self.tfbuf = Buffer(); self.tfl = TransformListener(self.tfbuf, self)
        self.sub = self.create_subscription(LaserScan, "/scan", self.cb, 10)
        self.pub = self.create_publisher(LaserScan, "/scan_filtered", 10)

    # -------------------------------------------------
    def cb(self, scan: LaserScan):
        try:
            tf: TransformStamped = self.tfbuf.lookup_transform(
                self.box_frame, scan.header.frame_id,
                rclpy.time.Time(seconds=0), rclpy.duration.Duration(seconds=0.05))
        except Exception as e:
            self.get_logger().warn_throttle(5.0, f"TF lookup failed: {e}")
            return

        # Ranges → Cartesian in laser frame --------------------------
        angles = scan.angle_min + np.arange(len(scan.ranges))*scan.angle_increment
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        zs = np.zeros_like(xs)

        # Transform into box_frame -----------------------------------
        q = tf.transform.rotation
        Rz = 2*(q.w*q.z + q.x*q.y)
        Rw = 1 - 2*(q.y*q.y + q.z*q.z)
        yaw = math.atan2(Rz, Rw)
        c,s = math.cos(yaw), math.sin(yaw)
        x_b = c*xs - s*ys + tf.transform.translation.x
        y_b = s*xs + c*ys + tf.transform.translation.y
        z_b = zs                   + tf.transform.translation.z

        # mask inside the box ----------------------------------------
        m = self.box
        inside =  (m[0] <= x_b) & (x_b <= m[1]) & \
                  (m[2] <= y_b) & (y_b <= m[3]) & \
                  (m[4] <= z_b) & (z_b <= m[5])
        ranges[inside] = float('inf')        # or 0.0 to drop the hit

        # publish ----------------------------------------------------
        out = scan
        out.ranges = ranges.tolist()
        self.pub.publish(out)

# ------------------ main ------------------
def main():
    rclpy.init(); rclpy.spin(BoxFilter()); rclpy.shutdown()

if __name__ == "__main__":
    main()
