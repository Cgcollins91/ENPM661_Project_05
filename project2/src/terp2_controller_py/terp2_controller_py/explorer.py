#!/usr/bin/env python3
import rclpy, math, numpy as np
from rclpy.node            import Node
from nav_msgs.msg          import OccupancyGrid
from geometry_msgs.msg     import PoseStamped
from rcl_interfaces.srv    import SetParameters          # service used by ROS 2 parameter API
from rclpy.parameter       import Parameter
from std_msgs.msg import Bool

FREE = 0; UNKNOWN = -1
SCAN_RADIUS = 3.0      # m around robot to look for frontiers

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        # --- subscribe to the live map ---
        self.create_subscription(OccupancyGrid, "/clean_map", self.map_cb, 10)
        self.map : OccupancyGrid | None = None

        # --- service client that will set /controller_py parameters ---
        self.param_cli = self.create_client(SetParameters,
                                            "/controller_py/set_parameters")
        self.create_subscription(
            Bool, "/controller_py/goal_reached",
            self._reached_cb, 10)
        self.set_controller_goal([0.0, 0.0])
        self.goal_reached = True          # start “free to send”

        # ONLY FOR TESTING AND TUNING SLAM PARMS
        self.point_1 = [7.0, 0.0]
        self.point_2 = [-7.0, 0.0]
        self.tune_point = 'point1'
        self.TUNE_MODE = True


    # =========================================================
    #  Incoming map → choose a frontier cell → push to controller
    # =========================================================

    def _reached_cb(self, msg: Bool):
        self.goal_reached = msg.data

    def map_cb(self, msg: OccupancyGrid):
        self.map = msg

        if not self.goal_reached:          # robot still driving → do nothing
            return
        if self.TUNE_MODE and self.goal_reached and self.tune_point=='point1':
            self.tune_point='point2'
        elif self.TUNE_MODE and self.goal_reached and self.tune_point=='point2':
            self.tune_point='point1'

            
        if self.TUNE_MODE and self.tune_point=='point1':
            goal_str = self.point_1
        elif self.TUNE_MODE and self.tune_point=='point2':
            goal_str = self.point_2
        else:
            goal_xy = self.pick_frontier_goal()
            x, y = goal_xy
            goal_str = f"[{x:.3f}, {y:.3f}]"
        self.set_controller_goal(goal_str)

    # ---------------------------------------------------------
    def pick_frontier_goal(self):
        if self.map is None:
            return None

        # reshape flat int8 list into H×W array
        data = np.array(self.map.data, dtype=np.int8).reshape(
                    (self.map.info.height, self.map.info.width))

        # Frontier definition: FREE cell that touches at least one UNKNOWN neighbour
        free     = data == FREE
        unknown  = data == UNKNOWN

        # 4-connected neighbours
        neigh = (
            np.roll(unknown,  1, axis=0) | np.roll(unknown, -1, axis=0) |
            np.roll(unknown,  1, axis=1) | np.roll(unknown, -1, axis=1)
        )
        frontier_mask = np.logical_and(free, neigh)

        ys, xs = np.where(frontier_mask)
        if len(xs) == 0:
            self.get_logger().info("No frontier left — exploration done ")
            rclpy.shutdown()
            return None

        # pick one at random (anything smarter is fine too)
        idx = np.random.randint(len(xs))
        cell_x, cell_y = xs[idx], ys[idx]

        # convert grid index ➜ metres in map frame
        world_x = cell_x * self.map.info.resolution + self.map.info.origin.position.x
        world_y = cell_y * self.map.info.resolution + self.map.info.origin.position.y
        return world_x, world_y

    # ---------------------------------------------------------
    def set_controller_goal(self, goal: str):
        """
        Send the /controller_py “goal” parameter as a double_array.

        Parameters
        ----------
        goal : Union[str, Sequence[float]]
            • "[1.2, -0.8]"   (string with brackets / commas)  **or**
            • (1.2, -0.8)     (list / tuple of floats)
        """
        # ---------- normalise input to a list[float] ----------
        if isinstance(goal, str):
            try:
                vals = [float(v) for v in goal.strip("[]() ").split(",")]
            except ValueError:
                self.get_logger().error(f"Goal string malformed: {goal!r}")
                return
        else:
            vals = [float(v) for v in goal]

        # ---------- build the SetParameters request ----------
        req = SetParameters.Request()
        req.parameters.append(
            Parameter(
                name="goal",
                value=vals,
                type_=Parameter.Type.DOUBLE_ARRAY          # force correct type
            ).to_parameter_msg()
        )

        # ---------- call the service ----------
        if not self.param_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("/controller_py parameter service not available")
            return

        future = self.param_cli.call_async(req)

        # ---------- result callback ----------
        def _done_cb(fut):
            try:
                res = fut.result()
                if res.results and res.results[0].successful:
                    self.get_logger().info(f"Sent new goal {vals} ✅")
                else:
                    reason = res.results[0].reason if res.results else "unknown error"
                    self.get_logger().warn(f"Goal rejected: {reason}")
            except Exception as e:
                self.get_logger().error(f"Set-parameter call failed: {e}")

        future.add_done_callback(_done_cb)


# ---------- main ----------
def main():
    rclpy.init()
    rclpy.spin(FrontierExplorer())

if __name__ == "__main__":
    main()