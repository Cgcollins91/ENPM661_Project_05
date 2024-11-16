#!/usr/bin/python3

# Project: ENPM662-Project1-Group1
# License: MIT
# The code in this file represents the collective work of Group 1.
# At times, code has been liberally borrowed from files provided
# with the project's instructions or from OSRF training materials.
# Please see the project report for a list of references, or contact
# the team with specific questions.

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
import matplotlib.pyplot as plt
import signal
import sys
import math
from collections import namedtuple

pose = namedtuple('pose', ['x', 'y', 'theta'])

class ModelPlotter(Node):
    def __init__(self):
        super().__init__('model_plotter')

        self.plot_data = []
        
        # Subscriber to the /model_states topic
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10)
        self.subscription  # prevent unused variable warning

    def model_states_callback(self, msg):
        # Loop through all models and print their names and poses
        for i, model_name in enumerate(msg.name):
            position    = msg.pose[i].position
            orientation = msg.pose[i].orientation

            if model_name == 'terp1':
                theta = quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
                theta += 90.0
                if theta < 0.0:
                    theta += 360.0
                p = pose(position.x, position.y, theta)
                self.plot_data.append(p)
                self.get_logger().info(
                    f'Model: {model_name}\n'
                    f'  Position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}\n'
                    f'  Orientation: x={orientation.x:.4f}, y={orientation.y:.4f}, z={orientation.z:.4f}, w={orientation.w:.4f}, theta={theta:.2f}'
                )

    def plot_pose(self):
        plot_x = []
        plot_y = []
        for p in self.plot_data:
            plot_x.append(p.x)
            plot_y.append(p.y)
            self.get_logger().info(f'plotting {p.x:.1f}, {p.y:.1f}, {p.theta:.1f}')
            rad_theta = p.theta * math.pi / 180.0
            plt.quiver(p.x, p.y, math.cos(rad_theta), math.sin(rad_theta), angles='xy', scale_units='xy', scale=2, color='r', width=0.005, zorder=2)
        plt.plot(plot_x, plot_y, color='b', linestyle=':', linewidth=1, zorder=1)
        plt.axis('equal')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title("Robot Trajectory")
        save_path = 'pose_plot.png'
        plt.savefig(save_path)
        return

def quaternion_to_yaw(x, y, z, w):
    theta = math.atan2(2 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return theta * 180.0 / math.pi

def signal_handler(sig, frame):
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    model_plotter = ModelPlotter()

    # Register the signal handler
    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(model_plotter)
    except KeyboardInterrupt:
        pass
    finally:
        model_plotter.plot_pose()
    model_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
