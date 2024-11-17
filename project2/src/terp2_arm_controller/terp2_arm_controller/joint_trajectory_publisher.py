import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import SetBool
import numpy as np


class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',  # Controller's command topic
            10
        )
        self.timer_period = 0.1  # Publish every 0.01 seconds (100 Hz)
        self.timer = self.create_timer(self.timer_period, self.publish_joint_trajectory)

        self.target_service = self.create_service(
            SetBool,
            'set_target_positions',
            self.set_target_positions
        )

        # Joint names (ensure these match your robot's joints)
        self.joint_names = [
            'joint_arm_1',
            'joint_arm_2',
            'joint_arm_3',
            'joint_arm_4',
            'joint_arm_5'
        ]

        # Initial target joint positions
        self.target_joint_positions = {
            'joint_arm_1': 0.0,
            'joint_arm_2': np.pi / 4,
            'joint_arm_3': -np.pi / 2,
            'joint_arm_4': 0.0 ,
            'joint_arm_5': 0.0
        }

        # Initialize current joint positions
        self.current_joint_positions = [0.0] * len(self.joint_names)

        # Step sizes for smooth interpolation
        self.joint_step = {
            'joint_arm_1': 0.1,
            'joint_arm_2': 0.1,
            'joint_arm_3': 0.1,
            'joint_arm_4': 0.1,
            'joint_arm_5': 0.01
        }

        self.start_time = self.get_clock().now()
        self.moving = False  # Flag to indicate if joints are moving

    def update_joint_positions(self):
        """Update current joint positions towards target positions."""
        moving = False
        for i, joint in enumerate(self.joint_names):
            step = self.joint_step[joint]
            current_position = self.current_joint_positions[i]
            target_position  = self.target_joint_positions[joint]

            if abs(current_position - target_position) > step:
                self.current_joint_positions[i] += step * np.sign(target_position - current_position)
                self.get_logger().info(f"Joint {joint} moving to {self.current_joint_positions[i]}")
    
                moving = True
            else:
                self.current_joint_positions[i] = target_position
        self.moving = moving

    def publish_joint_trajectory(self):
        """Publish the joint trajectory to the controller."""
        # Update joint positions
        self.update_joint_positions()

        # Create JointTrajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Create a trajectory point
        point = JointTrajectoryPoint()
        point.positions = self.current_joint_positions
        point.time_from_start.sec = 5  # Time to reach the positions

        trajectory_msg.points.append(point)

        # Publish the trajectory
        self.publisher.publish(trajectory_msg)

        # Log if moving
        if self.moving:
            self.get_logger().info(f"Publishing joint trajectory: {trajectory_msg}")

    def set_target_positions(self, request, response):
        """Service callback to update target joint positions."""
        # Example: Update all joints to a new target
        new_target = np.pi / 4 if request.data else 0.0
        for joint in self.joint_names:
            self.target_joint_positions[joint] = new_target
        response.success = True
        response.message = "Updated target positions"
        self.get_logger().info(f"Target positions updated to {new_target}")
        return response


def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_publisher = JointTrajectoryPublisher()
    rclpy.spin(joint_trajectory_publisher)

    # Cleanup
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
