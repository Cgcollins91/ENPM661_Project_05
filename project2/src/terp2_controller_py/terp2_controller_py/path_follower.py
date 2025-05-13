#!/usr/bin/env python3
# path_follower.py  –  sends waypoints to Terp2Controller and pauses at goals
#
# • reads  path.csv   (all waypoints)
# • reads  goals.csv  (subset that should trigger the pick-and-place)
# • waits for a clean False→True transition on /controller_py/goal_reached
#   before dispatching the next waypoint

import csv, os, time
from typing import List
import math
import rclpy
from rclpy.node          import Node
from std_msgs.msg        import Bool
from rcl_interfaces.srv  import SetParameters
from rcl_interfaces.msg  import Parameter as ParamMsg, ParameterValue, ParameterType
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool, Float64MultiArray

REACHED_THRESH = 0.15 # SET SAME AS controller_py.py

# ───────────────── helper functions ─────────────────
def load_waypoints(fname: str) -> list[list[float]]:
    """
    Return waypoints spaced >= min_step apart.
    • Skips blank / comment lines
    • Skips any point closer than min_step to the previous *kept* point
    """
    wps   = []
    last  = None                            # last point we DID keep

    with open(fname, newline="") as f:
        reader    = csv.reader(f)
        header    = next(reader, None)         # skip header if present
        first_row = next(reader, None)         # skip 0,0 node
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            x, y = map(float, row[:2])

            if last is None or math.hypot(x - last[0], y - last[1]) >= 2*REACHED_THRESH :
                wps.append([x, y])
                last = (x, y)               # update the reference

    return wps


def load_goals(fname: str) -> List[List[float]]:
    goals = []
    with open(fname, newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            x, y = map(float, row[:2])
            goals.append([x, y])
    return goals


# ───────────────── main node ─────────────────
class PathFollower(Node):
    GOAL_EPS = 1e-3                      # metres, tolerance for matching

    def __init__(self):
        super().__init__("path_follower")

        pkg_share     = get_package_share_directory("terp2_controller_py")
        path_csv      = os.path.join(pkg_share, "path", "path.csv")
        goals_csv     = os.path.join(pkg_share, "path", "goals.csv")

        self.waypoints    = load_waypoints(path_csv)
        self.goal_set     = {(gx, gy) for gx, gy in load_goals(goals_csv)}

        self.idx           = -1
        self.this_is_goal  = False
        self.goal_active   = False    # True after we send a goal, False once handled
        self.need_false    = False    # require one False before trusting the next True

        self.param_cli = self.create_client(SetParameters,
                                            "/controller_py/set_parameters")
        self.get_logger().info("Waiting for /controller_py/set_parameters…")
        self.param_cli.wait_for_service()

        self.create_subscription(Bool,
                                 "/controller_py/goal_reached",
                                 self._reached_cb,
                                 10)
        self.base_vel_pub = self.create_publisher(
            Float64MultiArray, "/velocity_controller/commands", 10
        )
        self._send_next_wp()

    # ───────── waypoint dispatch ─────────
    def _send_next_wp(self):
        self.idx += 1
        if self.idx >= len(self.waypoints):
            self.get_logger().info("🎉  All waypoints completed.")
            rclpy.shutdown()
            return

        wp = self.waypoints[self.idx]
        x, y = wp
        self.this_is_goal = any(abs(x - gx) < self.GOAL_EPS and
                                abs(y - gy) < self.GOAL_EPS
                                for gx, gy in self.goal_set)

        self.goal_active = True     # expect a fresh reached signal for this wp
        self.need_false  = True

        self.get_logger().info(
            f"Sending wp {self.idx+1}/{len(self.waypoints)} → {wp} "
            f"{'(GOAL)' if self.this_is_goal else ''}"
        )

        param = ParamMsg(
            name="goal",
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=wp,
            ),
        )
        fut = self.param_cli.call_async(SetParameters.Request(parameters=[param]))
        fut.add_done_callback(self._param_response)

    def _param_response(self, fut):
        if not fut.result() or not fut.result().results[0].successful:
            self.get_logger().error("Failed to set goal parameter; aborting.")
            rclpy.shutdown()
        else:
            self.get_logger().info("Waypoint accepted.")

    # ───────── goal-reached callback ─────────
    def _reached_cb(self, msg: Bool):
        # Ignore if no goal is active
        if not self.goal_active:
            return

        # First wait until we have observed at least one False
        if self.need_false:
            if not msg.data:
                self.need_false = False       # got the False, now we trust True
            return

        # Now only act on True edges
        if not msg.data:
            return

        # Valid reached event
        self.get_logger().info("Goal reached ")

        if self.this_is_goal:
            self.get_logger().info("Executing Book placement")
            self._run_ros_sequence()

        self.goal_active = False
        self._send_next_wp()

    def _freeze_base(self):
        """Continuously send zero velocity to hold the mobile base steady."""
        self.base_vel_pub.publish(Float64MultiArray(data=[0.0, 0.0]))

    # ------------------------------------------------------------
    #  Remove one book from storage, then place it on the shelf
    # ------------------------------------------------------------
    def _run_ros_sequence(self):
    # helper to send one arm pose then wait
        def send_arm(joints, wait_s):
            self._set_param("arm_goal", joints)
            self._freeze_base()
            time.sleep(wait_s)

        # helper to set gripper then wait
        def send_gripper(pads_open_cm, wait_s):
            pad = pads_open_cm
            self._set_param("gripper_goal", [0.0, pad, pad, pad])
            self._freeze_base()
            time.sleep(wait_s)

        # ------------ remove 1× book from storage ------------
        send_arm([0.0,  -0.8,  0.0,  0.0,  0.0], 1)
        send_arm([-1.1, -1.1,  0.0, -0.1,  0.0], 1)
        send_arm([-1.25, -0.9, -0.3, -0.1, 0.0], 1)
        send_gripper(0.03, 1)                              # open pads ~3 cm
        send_arm([-1.1, -1.1,  0.0, -0.1,  0.0], 1)
        send_arm([0.0,  -0.8,  0.0,  0.0,  0.0], 1)
        send_arm([0.0,   0.0,  0.0,  0.0,  0.0], 1)

        # ------------ place 1× book on shelf ------------
        send_arm([0.0,  0.0,  0.0,  0.0,  0.0], 2)
        send_arm([0.0,  1.0,  2.1,  3.0,  0.0], 2)
        send_arm([0.2,  1.2,  2.2,  3.0,  0.0], 1)
        send_gripper(0.0, 1)                               # close pads
        send_arm([0.2,  1.2,  2.2,  3.0,  0.0], 2)
        send_arm([0.0,  0.0,  0.0,  0.0,  0.0], 2)

        time.sleep(0.5)    # settle before next waypoint

    def _set_param(self, name: str, values: List[float]):
        msg = ParamMsg(
            name=name,
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=values,
            ),
        )
        self.param_cli.call_async(SetParameters.Request(parameters=[msg]))


# ───────────────── entry point ─────────────────
def main():
    rclpy.init()
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
