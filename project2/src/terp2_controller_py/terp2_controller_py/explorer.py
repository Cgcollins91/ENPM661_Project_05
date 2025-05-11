#!/usr/bin/env python3
import math, heapq, rclpy, numpy as np
from rclpy.node                import Node
from rclpy.qos                 import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg              import OccupancyGrid, Odometry
from std_msgs.msg              import Bool, Float32
from rcl_interfaces.srv        import SetParameters
from rclpy.parameter           import Parameter
from scipy.ndimage             import label, binary_dilation

# ─── map conventions ────────────────────────────────────────────────────
FREE, UNKNOWN, OCC_VAL = 0, -1, 100
INFLATE_M   = 0.35            # extra wall clearance (m)
SCAN_RADIUS = 5.0             # “local” search radius (m)
MIN_UNKNOWN_IN_CLUSTER = 15
VISITED_EPS = 0.50            # don’t revisit a centroid closer than this (m)
REACH_EPS   = 0.45            # counts as reached in *all* checks (m)
MAX_ASTAR_CELLS = 30_000      # bail-out limit for A*

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        # latched map subscription
        qos_latched = QoSProfile(depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(OccupancyGrid, "/clean_map",
                                 self.map_cb, qos_latched)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 20)
        self.create_subscription(Bool, "/controller_py/goal_reached",
                                 self.reached_cb, 10)
        self.create_subscription(Float32, "/controller_py/goal_dist",
                                 self.dist_cb, 10)

        self.param_cli = self.create_client(SetParameters,
                                            "/controller_py/set_parameters")

        # ── state ─────────────────────────────────────────────────────
        self.map : OccupancyGrid | None = None
        self.robot_xy : tuple[float, float] | None = None
        self.goal_in_flight = False
        self.goal_dist = float("inf")
        self.current_goal : tuple[float, float] | None = None
        self.visited : list[tuple[float, float]] = []

    # ───────────────────────── callbacks ───────────────────────────────
    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.robot_xy = (p.x, p.y)

    def dist_cb(self, msg: Float32):
        self.goal_dist = msg.data
        if self.goal_in_flight and self.goal_dist < REACH_EPS:
            self.get_logger().debug(
                f"auto-reach via /goal_dist ({self.goal_dist:.2f} m ≤ {REACH_EPS})")
            self.goal_in_flight = False

    def reached_cb(self, msg: Bool):
        if msg.data:
            self.goal_in_flight = False
            if self.current_goal:
                self.visited.append(self.current_goal)
                if len(self.visited) > 50:
                    self.visited.pop(0)

    def map_cb(self, msg: OccupancyGrid):
        self.map = msg

        # odom-based auto-reach (runs before early return)
        # if self.goal_in_flight and self.current_goal and self.robot_xy:
        #     if math.hypot(self.robot_xy[0] - self.current_goal[0],
        #                   self.robot_xy[1] - self.current_goal[1]) < REACH_EPS:
        #         self.get_logger().debug(
        #             f"odom-auto-reach (≤{REACH_EPS} m)")
        #         self.goal_in_flight = False

        # if still “busy” or no pose yet, do nothing
        if self.goal_in_flight or self.robot_xy is None:
            return

        # pick frontier and dispatch
        goal = self.pick_frontier_goal()
        if goal:
            self.set_controller_goal(goal)
            self.goal_in_flight = True
            self.current_goal   = goal

    # ───────────────────── frontier selection ──────────────────────────
    def pick_frontier_goal(self):
        if self.robot_xy is None:
            return None

        h, w   = self.map.info.height, self.map.info.width
        res    = self.map.info.resolution
        origin = self.map.info.origin.position
        grid   = np.asarray(self.map.data, dtype=np.int16).reshape((h, w))

        # inflate obstacles
        occ = (grid == OCC_VAL)
        pad = int(math.ceil(INFLATE_M / res))
        if pad > 0:
            occ = binary_dilation(occ, iterations=pad)
        free    = (grid == FREE) & (~occ)
        unknown = (grid == UNKNOWN)
        neigh   = (np.roll(unknown, 1, 0) | np.roll(unknown, -1, 0) |
                   np.roll(unknown, 1, 1) | np.roll(unknown, -1, 1))
        frontier = free & neigh
        if not frontier.any():
            self.get_logger().info("🎉 Exploration finished")
            return None

        labels, n = label(frontier, structure=[[0, 1, 0], [1, 1, 1], [0, 1, 0]])
        rx, ry = self.robot_xy
        rcx = int((rx - origin.x) / res)
        rcy = int((ry - origin.y) / res)

        # first try within SCAN_RADIUS; if none found, try entire map
        goal = self._score_frontiers(labels, n, rx, ry, rcx, rcy,
                                     res, origin, free, scan_limit=SCAN_RADIUS)
        if goal is not None:
            return goal

        return self._score_frontiers(labels, n, rx, ry, rcx, rcy,
                                     res, origin, free, scan_limit=float("inf"))

    # helper: evaluate clusters and return best centroid or None
    def _score_frontiers(self, labels, n, rx, ry, rcx, rcy,
                         res, origin, free_mask, scan_limit):
        best_score, best_goal = -1.0, None

        for idx in range(1, n + 1):
            ys, xs = np.where(labels == idx)
            if len(xs) < MIN_UNKNOWN_IN_CLUSTER:
                continue

            cx, cy = xs.mean(), ys.mean()
            gx = cx * res + origin.x
            gy = cy * res + origin.y

            dist = math.hypot(gx - rx, gy - ry)
            if dist < REACH_EPS or dist > scan_limit:
                continue
            if any(math.hypot(gx - vx, gy - vy) < VISITED_EPS
                   for vx, vy in self.visited):
                continue
            if not self.reachable((rcx, rcy), (int(cx), int(cy)), free_mask):
                continue

            score = len(xs) / (dist + 1e-3)      # information gain per metre
            if score > best_score:
                best_score, best_goal = score, (gx, gy)

        return best_goal

    # ────────────────────── A* reachability ────────────────────────────
    def reachable(self, start, goal, free_mask):
        h, w = free_mask.shape
        inside = lambda c: 0 <= c[0] < w and 0 <= c[1] < h
        if not (inside(goal) and free_mask[goal[1], goal[0]]):
            return False

        openh = [(0, start)]
        g_cost = {start: 0}
        explored = 0

        while openh:
            f, (x, y) = heapq.heappop(openh)
            if (x, y) == goal:
                return True
            explored += 1
            if explored > MAX_ASTAR_CELLS:
                return False

            for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if not inside((nx, ny)) or not free_mask[ny, nx]:
                    continue
                ng = g_cost[(x, y)] + 1
                if ng < g_cost.get((nx, ny), 1e9):
                    g_cost[(nx, ny)] = ng
                    h_est = abs(nx - goal[0]) + abs(ny - goal[1])
                    heapq.heappush(openh, (ng + h_est, (nx, ny)))

        return False  # open list exhausted

    # ───────────────── controller interface ────────────────────────────
    def set_controller_goal(self, xy):
        req = SetParameters.Request()
        req.parameters.append(
            Parameter(name="goal", value=list(xy)).to_parameter_msg())

        if not self.param_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warning("controller_py parameter service unavailable")
            return

        fut = self.param_cli.call_async(req)
        fut.add_done_callback(lambda _: self.get_logger().info(f"New goal → {xy}"))

# ───────────────────────────── main ────────────────────────────────────
def main():
    rclpy.init()
    rclpy.spin(FrontierExplorer())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
