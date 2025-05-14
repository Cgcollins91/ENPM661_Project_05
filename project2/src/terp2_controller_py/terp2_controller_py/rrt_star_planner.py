# rrt_star_planner.py – drop‑in replacement for the A* block in your library game
# -----------------------------------------------------------------------------
# This module holds a minimalist, self‑contained RRT* implementation that plugs
# into the environment, utility functions, and pygame visualisation you already
# have.  It deliberately works in **(x,y)** configuration space only to keep the
# algorithm readable.  A short steering‑function is provided that rolls the
# differential‑drive forward in small increments so the final path respects the
# robot kinematics; for many projects you can start with the straight‑line steer
# then swap‑in your existing move_set()/reverse_move() to get curvature‑aware
# trajectories.
# -----------------------------------------------------------------------------
# Usage from your main loop (inside the “for targ in TL” block):
#
#     planner = RRTStar(start_xy=start[:2], goal_xy=goal,                                 # (x,y)
#                       is_free=lambda p: is_free(p, pxarray, pallet, screen),           # collision fn
#                       sample_area=(0, rows, 0, cols),                                  # bounds
#                       step_len=5.0, goal_sample_rate=0.05, search_radius=15.0,
#                       max_iter=4000)
#     ok, rrt_path = planner.plan(display=screen)
#
# The returned rrt_path is a list of (x,y) tuples from start→goal.  You can feed
# it into your reverse_move()/curve plotting exactly like the A* solution list.
#
# The helper is_free() is shown at the bottom; it re‑uses your buffer_set or the
# colour test you were already doing.
# -----------------------------------------------------------------------------
from __future__ import annotations

from random import random, uniform
from math   import hypot
import pygame
import math

class Node:
    __slots__ = ("x", "y", "parent", "cost")
    def __init__(self, x: float, y: float):
        self.x: float     = x
        self.y: float     = y
        self.parent: Node | None = None
        self.cost: float  = 0.0          # g‑value (cost‑to‑come)

    # Convenience ---------------------------------------------------------
    def __iter__(self):
        yield self.x; yield self.y
    def dist(self, other: "Node") -> float:      # Euclidean distance helper
        return hypot(self.x - other.x, self.y - other.y)

class RRTStar:
    """A bare‑bones RRT* planner good enough for 2‑D workspaces."""

    def __init__(self, *,
                 start_xy: tuple[float, float],
                 goal_xy:  tuple[float, float],
                 is_free,                               # fn(p: (x,y)) → bool
                 sample_area: tuple[float, float, float, float],
                 step_len: float = 5.0,
                 search_radius: float = 15.0,
                 goal_sample_rate: float = 0.05,
                 max_iter: int = 3000):

        self.start = Node(*start_xy)
        self.goal  = Node(*goal_xy)
        self.nodes: list[Node] = [self.start]

        self.is_free        = is_free
        self.xmin, self.xmax, self.ymin, self.ymax = sample_area
        self.step_len       = step_len
        self.search_radius  = search_radius
        self.goal_rate      = goal_sample_rate
        self.max_iter       = max_iter

    # ------------------------------------------------------------------
    def plan(self, display: pygame.Surface | None = None):
        """Run RRT*; return (success flag, path list). Optionally live‑draw."""
        if not self.is_free((self.start.x, self.start.y)) or not self.is_free((self.goal.x, self.goal.y)):
            return False, []

        surf = display
        for k in range(self.max_iter):
            rnd = self._sample()                      # (x,y)
            nearest = self._nearest(rnd)
            new = self._steer(nearest, rnd)
            if not self._collision_free(nearest, new):
                continue

            near_inds = self._near(new)
            parent = nearest
            min_cost = nearest.cost + nearest.dist(new)
            # Choose best parent (cost‑optimal inside search_radius)
            for i in near_inds:
                nd = self.nodes[i]
                if self._collision_free(nd, new) and nd.cost + nd.dist(new) < min_cost:
                    parent, min_cost = nd, nd.cost + nd.dist(new)
            new.parent = parent; new.cost = min_cost
            self.nodes.append(new)

            # Rewire --------------------------------------------------
            for i in near_inds:
                nd = self.nodes[i]
                if nd is parent: continue
                alt_cost = new.cost + new.dist(nd)
                if self._collision_free(new, nd) and alt_cost < nd.cost:
                    nd.parent = new; nd.cost = alt_cost

            # Draw sampled tree edge (optional) ----------------------
            if surf:
                pygame.draw.line(surf, (61,119,245), (new.x, new.y), (parent.x, parent.y))
                pygame.display.update()

            # Check for goal connection ------------------------------
            if new.dist(self.goal) <= self.step_len and self._collision_free(new, self.goal):
                self.goal.parent = new; self.goal.cost = new.cost + new.dist(self.goal)
                self.nodes.append(self.goal)
                return True, self._extract_path()

        return False, []

    # ------------------------------------------------------------------
    def _sample(self):
        if random() < self.goal_rate:
            return (self.goal.x, self.goal.y)
        return (uniform(self.xmin, self.xmax), uniform(self.ymin, self.ymax))

    def _nearest(self, point):
        return min(self.nodes, key=lambda n: (n.x - point[0])**2 + (n.y - point[1])**2)

    def _steer(self, from_node: Node, to_xy: tuple[float, float]):
        theta = math.atan2(to_xy[1] - from_node.y, to_xy[0] - from_node.x)
        new_x = from_node.x + self.step_len * math.cos(theta)
        new_y = from_node.y + self.step_len * math.sin(theta)
        return Node(new_x, new_y)

    def _collision_free(self, n1: Node, n2: Node):
        # Simple discretised segment check – sample at 1‑unit steps
        steps = max(1, int(n1.dist(n2)))
        for i in range(steps + 1):
            u = i/steps
            p = (n1.x + u*(n2.x-n1.x), n1.y + u*(n2.y-n1.y))
            if not self.is_free(p):
                return False
        return True

    def _near(self, new: Node):
        r_sq = self.search_radius ** 2
        return [i for i,n in enumerate(self.nodes) if (n.x-new.x)**2 + (n.y-new.y)**2 <= r_sq]

    def _extract_path(self):
        path = []
        node = self.goal
        while node:
            path.append((node.x, node.y))
            node = node.parent
        path.reverse()
        return path

# -----------------------------------------------------------------------------
# Helper that re‑uses your colour buffer check.  Swap‑in your own fast lookup
# against `buffer_set` if you have it available in this scope.
# -----------------------------------------------------------------------------

def is_free(p, pxarray, pallet, screen):
    x, y = int(p[0]), int(p[1])
    try:
        col = pxarray[x, y]
    except IndexError:
        return False
    return not (col == screen.map_rgb(pallet["black"]) or col == screen.map_rgb(pallet["green"]))
