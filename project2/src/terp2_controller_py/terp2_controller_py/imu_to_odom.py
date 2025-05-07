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
        

    def cb(self, imu: Imu):
         # ---------------- Odometry message ----------------
        odom              = Odometry()
        odom.header.stamp = imu.header.stamp
        odom.header.frame_id  = "odom"          # parent
        odom.child_frame_id   = "base_link"     # child

        odom.pose.pose.orientation = imu.orientation
        # leave position at 0,0,0  (pure heading sensor)
        self.pub.publish(odom)

        # ---------------- TF transform --------------------
        t                    = TransformStamped()
        t.header.stamp       = imu.header.stamp
        t.header.frame_id    = "odom"           # parent
        t.child_frame_id     = "base_link"      # child
        t.transform.rotation = imu.orientation
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        self.tf_br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(Imu2Odom())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
