#!/usr/bin/env python3
import rclpy, math, numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

FREE = 0; UNKNOWN = -1
SCAN_RADIUS = 3.0      # m around robot to look for frontiers

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_cb, 10)
        self.navigator = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.map = None

    # --------------------------------------------------------------
    def map_cb(self, msg: OccupancyGrid):
        self.map = msg
        goal = self.pick_frontier_goal()
        if goal:
            self.send_goal(goal)

    # --------------------------------------------------------------
    def pick_frontier_goal(self):
        if not self.map: return None
        data = np.array(self.map.data, dtype=np.int8).reshape(
                    (self.map.info.height, self.map.info.width))
        # frontier = FREE cell adjacent to UNKNOWN
        free = data == FREE
        unknown = data == UNKNOWN
        frontier_mask = np.logical_and(
            free,
            np.logical_or.reduce([np.roll(unknown, s, axes)
                for axes in [(0,1),(0,-1),(1,0),(-1,0)]]))
        ys, xs = np.where(frontier_mask)
        if len(xs) == 0:
            self.get_logger().info("No frontier left -- map completed 🎉")
            rclpy.shutdown(); return None

        # pick the frontier furthest from previous goal (cheap heuristic)
        idx = np.random.randint(len(xs))
        x_map, y_map = xs[idx], ys[idx]
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = \
            x_map * self.map.info.resolution + self.map.info.origin.position.x
        goal.pose.position.y = \
            y_map * self.map.info.resolution + self.map.info.origin.position.y
        goal.pose.orientation.w = 1.0   # face forward, doesn't matter
        return goal

    # --------------------------------------------------------------
    def send_goal(self, goal):
        if not self.navigator.wait_for_server(timeout_sec=0.5): return
        self.get_logger().info(f"Exploring frontier @({goal.pose.position.x:.1f}, "
                               f"{goal.pose.position.y:.1f})")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal
        self.navigator.send_goal_async(goal_msg)

# ---------- main ----------
def main():
    rclpy.init()
    rclpy.spin(FrontierExplorer())

if __name__ == "__main__":
    main()
