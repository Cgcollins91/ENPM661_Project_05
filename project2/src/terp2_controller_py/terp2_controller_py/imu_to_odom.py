#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class Imu2Odom(Node):
    def __init__(self):
        super().__init__("imu_to_odom")
        self.create_subscription(Imu, "/imu", self.cb, 10)
        self.pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_br = TransformBroadcaster(self)
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.last_time = None
        

    def cb(self, imu: Imu):
    # Time delta
        curr_time = imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = curr_time
            return
        dt = curr_time - self.last_time
        self.last_time = curr_time

         # Integrate acceleration to get velocity (assuming IMU aligned with robot axes)
        ax = imu.linear_acceleration.x
        ay = imu.linear_acceleration.y

        self.vx += ax * dt
        self.vy += ay * dt

        # Integrate velocity to get position
        self.x += self.vx * dt
        self.y += self.vy * dt

         # ---------------- Odometry message ----------------
        odom              = Odometry()
        odom.header.stamp = imu.header.stamp
        odom.header.frame_id  = "odom"          # parent
        odom.child_frame_id   = "base_link"     # child

        odom.pose.pose.orientation = imu.orientation
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        # leave position at 0,0,0  (pure heading sensor)
        self.pub.publish(odom)

        # ---------------- TF transform --------------------
        t                    = TransformStamped()
        t.header.stamp       = imu.header.stamp
        t.header.frame_id    = "odom"           # parent
        t.child_frame_id     = "base_link"      # child
        t.transform.rotation = imu.orientation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        self.tf_br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(Imu2Odom())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
