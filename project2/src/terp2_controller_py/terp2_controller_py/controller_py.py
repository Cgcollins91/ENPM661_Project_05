#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# Project: ENPM661-Project5-Group2
# License: MIT
#
# This is a one-for-one translation of the original C++ terp2_controller node
# into Python / rclpy.  Large chunks of the logic are intentionally kept
# identical so you can diff the two files line-by-line.
# ---------------------------------------------------------------------------

from __future__ import annotations

import math
import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.msg import ModelStates, LinkStates
from std_msgs.msg import Bool, Float32




REACHED_THRESH =  0.15      # metres – tweak to taste
REACHED_RATE   =  5.0      # Hz   – how often we evaluate

# ---------------------------------------------------------------------------#
# Helpers                                                                    #
# ---------------------------------------------------------------------------#
class PID:
    """Minimal PID controller identical to the one used in C++."""

    def __init__(self, kp: float = 0.0, ki: float = 0.0, kd: float = 0.0):
        self.set_k_values(kp, ki, kd)
        self.reset()


    # ------------------------------------------------------------------ #
    def set_k_values(self, kp: float, ki: float, kd: float) -> None:
        self.kp, self.ki, self.kd = float(kp), float(ki), float(kd)


    def reset(self) -> None:
        self._prev_err = 0.0
        self._integral = 0.0


    # ------------------------------------------------------------------ #
    def calculate(self, error: float, dt: float) -> float:
        self._integral += error * dt
        derivative = (error - self._prev_err) / dt if dt else 0.0
        self._prev_err = error
        return (
            self.kp * error
            + self.ki * self._integral
            + self.kd * derivative
        )


class Quaternion:
    """Enough quaternion maths for yaw extraction."""

    def __init__(self, w: float = 1.0, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.w, self.x, self.y, self.z = w, x, y, z

    # ------------------------------------------------------------------ #
    def yaw_deg(self) -> float:
        """Return yaw (Z-axis rotation) in degrees."""
        siny_cosp = 2.0 * (self.w * self.z + self.x * self.y)
        cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))


# ---------------------------------------------------------------------------#
# Main node                                                                   #
# ---------------------------------------------------------------------------#
class Terp2Controller(Node):
    def __init__(self) -> None:
        super().__init__("controller_py")

        # ---------------------------- ROS parameters -------------------- #
        self.declare_parameter("goal", [0.0, 0.0])
        self.declare_parameter("arm_goal", [0.0] * 5)
        self.declare_parameter("gripper_goal", [0.0] * 4)

        # ------------------------------ pubs ---------------------------- #
        self.pub_pos = self.create_publisher(
            Float64MultiArray, "/position_controller/commands", 10
        )
        self.pub_vel = self.create_publisher(
            Float64MultiArray, "/velocity_controller/commands", 10
        )
        self.pub_arm = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.pub_grip = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10
        )

        self.reached_pub = self.create_publisher(Bool,
                                                 "/controller_py/goal_reached",
                                                 qos_profile=1)
        
        self.pub_goal_dist = self.create_publisher(Float32,
                                            "/controller_py/goal_dist",
                                            10)

        # ----------------------------- subs ----------------------------- #
        self.create_subscription(
            LinkStates, "/gazebo/link_states", self.link_state_cb, 10
        )
        self.create_subscription(
            ModelStates, "/gazebo/model_states", self.model_state_cb, 10
        )

        # ------------------------- internal state ----------------------- #
        self.robot_id = "terp2"
        self.link_names: list[str] = []
        self.link_coords: list[list[float]] = []
        self.link_orients: list[Quaternion] = []

        self.position = [0.0, 0.0, 0.0]
        self.orientation = Quaternion()
        self.vel_linear = [0.0, 0.0, 0.0]
        self.vel_angular = [0.0, 0.0, 0.0]

        self.goal_xy = self.get_parameter("goal").value
        self.joint_goals = self.get_parameter("arm_goal").value
        self.gripper_goals = self.get_parameter("gripper_goal").value

        self.goal_radius = 0.0
        self.goal_theta = 0.0

        self.steer_max = math.radians(30)  # rad
        self.velocity_max = 5.0            # m/s wheel angular speed
        self.target_radius = 0.05          # m
        self.turn_radius = 0.4             # m
        self.dt = 0.5                      # s (matches timer)

        # PID objects
        self.pid_velocity = PID(5, 0.17, 1.7)
        self.pid_steer    = PID(1.7, 0.01, 0.1)

        # live outputs
        self.steer = 0.0
        self.velocity = 0.0

        # ---------------- parameter-set callback ----------------------- #
        self.add_on_set_parameters_callback(self.parameter_cb)

        # --------------------------- timers ----------------------------- #
        self.create_timer(self.dt, self.update_loop)

        self.get_logger().info("terp2_controller Python node ready")

    # =================================================================== #
    #                             Callbacks                               #
    # =================================================================== #
    def link_state_cb(self, msg: LinkStates) -> None:
        self.link_names.clear()
        self.link_coords.clear()
        self.link_orients.clear()
        for name, pose in zip(msg.name, msg.pose):
            self.link_names.append(name)
            self.link_coords.append(
                [pose.position.x * 1000, pose.position.y * 1000, pose.position.z * 1000]
            )
            self.link_orients.append(
                Quaternion(
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                )
            )

    # ------------------------------------------------------------------- #
    def model_state_cb(self, msg: ModelStates) -> None:
        try:
            idx = next(i for i, n in enumerate(msg.name) if n.startswith(self.robot_id))
        except StopIteration:
            self.get_logger().error("terp2 model not found in /model_states")
            return

        pose = msg.pose[idx]
        self.position = [pose.position.x, pose.position.y, pose.position.z]
        self.orientation = Quaternion(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        )

        twist = msg.twist[idx]
        self.vel_linear = [twist.linear.x, twist.linear.y, twist.linear.z]
        self.vel_angular = [twist.angular.x, twist.angular.y, twist.angular.z]

    # ------------------------------------------------------------------- #
    def parameter_cb(self, params: List[Parameter]) -> SetParametersResult:
        for p in params:
            if p.name == "goal":
                self.get_logger().info("LOCATION GOAL RECEIVED!")
                self.goal_xy = list(p.value)
                self.pid_velocity.reset()
                self.pid_steer.reset()

            elif p.name == "arm_goal":
                self.get_logger().info("ARM GOAL RECEIVED!")
                self.joint_goals = list(p.value)

            elif p.name == "gripper_goal":
                self.get_logger().info("GRIPPER GOAL RECEIVED!")
                self.gripper_goals = list(p.value)

            else:
                self.get_logger().warn("Unknown parameter set: %s", p.name)

        return SetParametersResult(successful=True)
    
    def _reached_cb(self):
        if self.goal_xy is None:                      # no active goal
            self.reached_pub.publish(Bool(data=True))
            return

        self.reached_pub.publish(Bool(data=(self.goal_radius < REACHED_THRESH)))
        self.get_logger().info(f"Distance to Goal {self.goal_radius}")

    # =================================================================== #
    #                         Control routines                            #
    # =================================================================== #
    def update_loop(self) -> None:
        self.set_goals()
        self.pid_update()
        self.robot_go()
        self._reached_cb()

    # ------------------------------------------------------------------- #
    def pid_update(self) -> None:
        # velocity PID on distance
        self.velocity = min(
            self.pid_velocity.calculate(self.goal_radius, self.dt), self.velocity_max
        )

        # steering PID on heading error
        self.steer = max(
            min(self.pid_steer.calculate(self.goal_theta, self.dt), self.steer_max),
            -self.steer_max,
        )

        # dead-band logic 
        if self.goal_radius < self.target_radius:
            self.steer = 0.0
            self.velocity = 0.0

        elif abs(self.goal_theta) > self.steer_max and self.goal_radius < self.turn_radius:
            self.steer = 0.0
            self.velocity = self.velocity_max
            self.get_logger().info("Going straight until viable turning radius…")


    # ------------------------------------------------------------------- #
    def robot_go(self) -> None:
        self.set_robot_steering(self.steer)
        self.set_robot_drive_wheels(self.velocity)
        self.set_robot_joint_thetas(self.joint_goals)
        self.set_robot_gripper_joints(self.gripper_goals)


    # =============== low-level set-helpers (publishers) ================= #
    def set_robot_drive_wheels(self, velocity: float) -> None:
        msg = Float64MultiArray(data=[velocity, velocity])
        self.pub_vel.publish(msg)


    def set_robot_steering(self, steer_angle: float) -> None:
        ratio = steer_angle / self.steer_max if self.steer_max else 0.0
        msg = Float64MultiArray(data=[-ratio, ratio])
        self.pub_pos.publish(msg)


    def set_robot_joint_thetas(self, goals: List[float]) -> None:
        jt = JointTrajectory()
        jt.joint_names = [f"joint_arm_{i}" for i in range(1, 6)]
        pt = JointTrajectoryPoint()
        pt.positions = goals
        pt.time_from_start.sec = 1
        jt.points.append(pt)
        self.pub_arm.publish(jt)


    def set_robot_gripper_joints(self, goals: List[float]) -> None:
        jt = JointTrajectory()
        jt.joint_names = [
            "joint_gripper_base",
            "joint_gripper_gear",
            "joint_gripper_pad1",
            "joint_gripper_pad2",
        ]
        pt = JointTrajectoryPoint()
        pt.positions = goals
        pt.time_from_start.sec = 1
        jt.points.append(pt)
        self.pub_grip.publish(jt)

    # =================================================================== #
    #                        Goal-calculation helpers                     #
    # =================================================================== #
    def set_goals(self) -> None:
        self.set_rotational_goal()
        self.set_distance_goal()

    def set_rotational_goal(self) -> None:
        yaw_goal = math.atan2(
            self.goal_xy[1] - self.position[1], self.goal_xy[0] - self.position[0]
        )
        yaw_goal = math.degrees(yaw_goal)
        if yaw_goal < 0:
            yaw_goal += 360.0

        orientation_angle = self.orientation.yaw_deg() + 90.0
        if orientation_angle < 0:
            orientation_angle += 360.0

        dangle = yaw_goal - orientation_angle
        if dangle < 0.0:
            dangle += 360.0

        if dangle > 180.0:
            dangle = 360.0 - dangle
            dangle *= -1.0

        self.goal_theta = math.radians(dangle)

    def set_distance_goal(self) -> None:
        self.goal_radius = math.hypot(
            self.goal_xy[0] - self.position[0], self.goal_xy[1] - self.position[1]
        )
        self.pub_goal_dist.publish(Float32(data=self.goal_radius))

    # =================================================================== #
    #                       Debug / logging helpers                       #
    # =================================================================== #
    def log_link_positions(self) -> None:
        for n, c in zip(self.link_names, self.link_coords):
            self.get_logger().info("%s → (%.2f, %.2f, %.2f)", n, *c)


# ---------------------------------------------------------------------------#
# main entry-point                                                           #
# ---------------------------------------------------------------------------#
def main() -> None:
    rclpy.init()
    node = Terp2Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

