#!/usr/bin/env python3
# model_state_to_odom.py

"""  –  bridge between Gazebo and robot_localization (Ground Truth)
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class GTBridge(Node):

    def __init__(self, model_name="terp2"):
        super().__init__("gt_bridge")
        self.model_name  = model_name
        self.pub_odom    = self.create_publisher(Odometry,  "/odom", 10)
        self.tf_bcaster  = TransformBroadcaster(self)
        self.sub_states  = self.create_subscription(ModelStates,
                                                    "/gazebo/model_states",
                                                    self.cb, 10)

    def cb(self, msg: ModelStates):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            self.get_logger().warn_once(f"Model {self.model_name!r} not in /model_states")
            return

        # ---------- build Odometry ----------
        odom            = Odometry()
        odom.header.stamp    = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"          # world-fixed frame
        odom.child_frame_id  = "base_link"     # robot frame

        odom.pose.pose     = msg.pose[idx]
        odom.twist.twist   = msg.twist[idx]

        # tiny covariances so robot_localization treats this as “almost perfect”
        odom.pose.covariance  = [1e-6] * 36
        odom.twist.covariance = [1e-6] * 36

        self.pub_odom.publish(odom)

        # ---------- publish TF ---------------
        tf              = TransformStamped()
        tf.header       = odom.header
        tf.child_frame_id     = "base_link"
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation      = odom.pose.pose.orientation
        self.tf_bcaster.sendTransform(tf)

def main():
    rclpy.init()
    rclpy.spin(GTBridge(model_name="terp2"))
    rclpy.shutdown()

if __name__ == "__main__":
    main()