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
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import select
import tty
import termios
from pynput import keyboard
import numpy as np

# Define key codes
LIN_VEL_STEP_SIZE = 1
ANG_VEL_STEP_SIZE = 0.05
ARM_STEP_SIZE = 0.1

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.settings = termios.tcgetattr(sys.stdin)

        self.arm_publisher = self.create_publisher( JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.joint_names = [
            'joint_arm_1',
            'joint_arm_2',
            'joint_arm_3',
            'joint_arm_4',
            'joint_arm_5'
        ]
        self.current_joint_positions = [0.0] * len(self.joint_names)
        self.reset_joint_steps()

    def reset_joint_steps(self):
        self.joint_step = {
            'joint_arm_1': 0.0,
            'joint_arm_2': 0.0,
            'joint_arm_3': 0.0,
            'joint_arm_4': 0.0,
            'joint_arm_5': 0.0
        }

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        self.msg = """
        Robot Teleop Control
        ---------------------------
        w a s d = Moving Around
        q = Halt Movement
        r f = Joint 1
        t g = Joint 2
        y h = Joint 3
        u j = Joint 4
        i k = Joint 5
        <ESC> = Quit
        """

        self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        linear_vel=0.0
        steer_angle=0.0
        self.reset_joint_steps()

        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  # Quit
                    linear_vel=0.0
                    steer_angle=0.0
                    self.reset_joint_steps()
                elif key == 'w':  # Forward
                    linear_vel += LIN_VEL_STEP_SIZE
                elif key == 's':  # Reverse
                    linear_vel -= LIN_VEL_STEP_SIZE
                elif key == 'd':  # Right
                    steer_angle -= ANG_VEL_STEP_SIZE
                elif key == 'a':  # Left
                    steer_angle += ANG_VEL_STEP_SIZE
                elif key == 'r':
                    self.joint_step['joint_arm_1'] += ARM_STEP_SIZE
                elif key == 'f':
                    self.joint_step['joint_arm_1'] -= ARM_STEP_SIZE
                elif key == 't':
                    self.joint_step['joint_arm_2'] += ARM_STEP_SIZE
                elif key == 'g':
                    self.joint_step['joint_arm_2'] -= ARM_STEP_SIZE
                elif key == 'y':
                    self.joint_step['joint_arm_3'] += ARM_STEP_SIZE
                elif key == 'h':
                    self.joint_step['joint_arm_3'] -= ARM_STEP_SIZE
                elif key == 'u':
                    self.joint_step['joint_arm_4'] += ARM_STEP_SIZE
                elif key == 'j':
                    self.joint_step['joint_arm_4'] -= ARM_STEP_SIZE
                elif key == 'i':
                    self.joint_step['joint_arm_5'] += ARM_STEP_SIZE
                elif key == 'k':
                    self.joint_step['joint_arm_5'] -= ARM_STEP_SIZE
                else:
                    continue


                if steer_angle>1.0:
                        steer_angle=1.0
                if steer_angle<-1.0:
                    steer_angle=-1.0

                print("Steer Angle\t",steer_angle)
                print("Linear Velocity\t",linear_vel)
                wheel_velocities.data = [-linear_vel,-linear_vel]
                joint_positions.data = [steer_angle,-steer_angle]

                self.joint_position_pub.publish(joint_positions)
                self.wheel_velocities_pub.publish(wheel_velocities)
                self.publish_joint_trajectory()
                self.reset_joint_steps()

    def update_joint_positions(self):
        for i, joint in enumerate(self.joint_names):
            step = self.joint_step[joint]
            current_position = self.current_joint_positions[i]
            if abs(step) > 0.0001:
                self.current_joint_positions[i] += step
        rounded_values = [round(value, 3) for value in self.current_joint_positions]
        self.get_logger().info(f'{rounded_values}')

    def publish_joint_trajectory(self):
        self.update_joint_positions()
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.current_joint_positions
        point.time_from_start.sec = 1  # Time to reach the positions

        trajectory_msg.points.append(point)
        self.arm_publisher.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
