#!/usr/bin/env python3

"""
simple_mapper.py    ROS 2 Humble node

*   Builds a 2-D occupancy-grid map from a filtered LaserScan
  ***Publishes***:
      • /map            full probability grid (0-100 int8, incl. unknown)
      • /clean_map      binary grid (–1/0/100) after morphological cleanup
      • /wall_segments  Marker.LINE_LIST with walls from a Hough transform
*   Broadcasts a static frame  map→odom  (identity for now) plus a
    periodic dynamic  map→odom  TF so SLAM / Nav 2 have the frame.
*   Stores “stable” occupied cells (hit ≥ STABLE_HITS times at >85 % prob)
    so line extraction sees persistent walls only.
"""

import rclpy, math, numpy as np
from rclpy.node           import Node
from sensor_msgs.msg      import LaserScan
from nav_msgs.msg         import OccupancyGrid, Odometry
from tf2_ros              import TransformBroadcaster, TransformListener, Buffer, StaticTransformBroadcaster   
from geometry_msgs.msg    import TransformStamped, Point
from rclpy.qos            import qos_profile_sensor_data
from scipy.ndimage import binary_erosion, binary_dilation
import cv2 
from visualization_msgs.msg import Marker

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ---------------- Fixed Parameters / Tunable ----------------------------------
MAP_RES     = 0.10                 # m / cell
MAP_SIZE    = int(20 // MAP_RES)   # side length in cells  (→ 20 m × 20 m map)

L0          = 0.0                  # log-odds prior
L_OCC       = 2.0                  # update increment for an occupied hit
L_FREE      = -1.0                 # update increment for a free cell

OCC_THRESH  = 0.85                 # probability → binary ‘occupied’
FREE_THRESH = 0.40                 # probability → binary ‘free’
OCC_STORE   = 0.85                 # prob required to start “stability” counter
STABLE_HITS = 3                    # consecutive hits before persisting as wall

latched = QoSProfile(
    reliability = ReliabilityPolicy.RELIABLE,
    durability  = DurabilityPolicy.TRANSIENT_LOCAL,
    depth = 1)
# ------------------------------------------------------- 


class Mapper(Node):
    """Occupancy-grid mapper + wall line plotter """
    def __init__(self):
        super().__init__("simple_mapper")

        # ---------- Map Storage ----------------------------------------------
        self.logodds = np.full((MAP_SIZE, MAP_SIZE), L0, dtype=np.float32)
        self.map_cx  = MAP_SIZE // 2        # world (0,0) at cell centre
        self.map_cy  = MAP_SIZE // 2

        self.hit_cnt  = np.zeros_like(self.logodds, dtype=np.uint8)
        self.wall_pts = set()   # {(cx,cy), …}  persisted cell indices

        # ---------- Publishers  -----------------------------------------
        self.map_pub   = self.create_publisher(OccupancyGrid, "/map", latched)
        self.clean_pub = self.create_publisher(OccupancyGrid, "/clean_map", latched)
        self.line_pub  = self.create_publisher(Marker, "/wall_segments", 1)

         # ----------  Transform helpers (map --> odom)   --------------------
        self.tf_buf    = Buffer()
        self.tf_lstn   = TransformListener(self.tf_buf, self)    # listener
        self.tf_br     = TransformBroadcaster(self)              # dynamic broadcaster
        self.static_br = StaticTransformBroadcaster(self)        # once-off broadcaster

        # Make map frame exist immediately
        t                      = TransformStamped()
        now                    = self.get_clock().now().to_msg()
        t.header.stamp         = now
        t.header.frame_id      = "map"
        t.child_frame_id       = "odom"       # Ground Truth Model State from Gazebo
        t.transform.rotation.w = 1.0          # identity quaternion
        self.static_br.sendTransform(t)


        # -------   Laser-scan subscription    -----------------------------
        self.first_scan_time = None
        self.create_subscription(
            LaserScan, "/scan_filtered", self.scan_cb, qos_profile_sensor_data
        )

        # periodic (dynamic) map→odom update 
        self.tf_timer = self.create_timer(0.5, self.pub_map_tf)

    # ----------------------------------------------------------------
  
  
    # --------------------    SCAN CALLBACK  ----------------------------
    def scan_cb(self, scan: LaserScan):
        """Ray-trace every valid range sample into the log-odds grid."""

         # Allow one scan for Transforms to initialize
        if self.first_scan_time is None:
            self.first_scan_time = self.get_clock().now()
            return     # skip one callback while Transforms  initialize
        
        # Get laser pose in odom frame
        try:
            tf = self.tf_buf.lookup_transform(
                            "odom", scan.header.frame_id,
                            rclpy.time.Time(), # 0 = latest transform
                            timeout=rclpy.duration.Duration(seconds=0.1))       
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        x, y, yaw = self.tf_to_xyyaw(tf)

        # Convert polar ray coordinates to Cartesian for every valid beam
        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment,
                           dtype=np.float32)
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        valid  = np.isfinite(ranges)
        angles, ranges = angles[valid], ranges[valid]

        ex = x + ranges * np.cos(angles + yaw)
        ey = y + ranges * np.sin(angles + yaw)

        #  (bx, by) = start of beam  (ox, xy) = end of beam
        for bx, by, ox, oy in zip(
                np.full_like(ex, x), np.full_like(ey, y), ex, ey):
            cbx, cby = self.world_to_cell(bx, by)
            cox, coy = self.world_to_cell(ox, oy)

            # if whole ray is outside of map ignore
            if not (self.inside(cbx, cby) or self.inside(cox, coy)):
                continue            

            self.update_cellray(bx, by, ox, oy)
        # Publish updated maps & line markers
        self.publish_map(scan.header.stamp)

    # -------------   Map / Marker Publishing Helpers -----------------------
    def publish_map(self, stamp):
            p = self.prob_grid()                     # 0-1 float array

            # Full probability map
            grid = self._make_grid_msg(stamp, p)
            self.map_pub.publish(grid)


            # Cleaned Binary Map
            clean_bin = self.clean_binary(self.bin_grid())    # int8 array
            clean = self._make_grid_msg(stamp, clean_bin / 100.0)
            clean.data = clean_bin.reshape(-1).tolist()       # overwrite with int8
            self.clean_pub.publish(clean)
            self.publish_lines(stamp)


    def publish_lines(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "map"
        marker.type   = Marker.LINE_LIST
        marker.scale.x = 0.02                 # 2 cm wide lines
        marker.color.r = 1.0; marker.color.g = 0.5
        marker.color.a = 1.0
        marker.action = Marker.ADD
        marker.id     = 0
        for (p1, p2) in self.detect_lines():
            marker.points.extend([Point(x=p1[0], y=p1[1], z=0.0),
                                Point(x=p2[0], y=p2[1], z=0.0)])
        self.line_pub.publish(marker)

    def pub_map_tf(self):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id  = "odom"
        # identity: no pose correction yet
        self.tf_br.sendTransform(t)        # <-- use broadcaster
    # ----------------------------------------------------------------


    # --------------------   Grid helpers ----------------------------
    def prob_grid(self):
        """Return float32 grid of P(occ) in [0,1]."""
        lo = np.clip(self.logodds, -50, 50, out=self.logodds)   # in-place, no new array
        return 1 - 1/(1+np.exp(lo))
    
    
    def _make_grid_msg(self, stamp, prob):
        msg = OccupancyGrid()
        msg.header.stamp    = stamp
        msg.header.frame_id = "map"
        msg.info.resolution = MAP_RES
        msg.info.width      = MAP_SIZE
        msg.info.height     = MAP_SIZE

        msg.info.origin.position.x    = -(MAP_SIZE // 2) * MAP_RES
        msg.info.origin.position.y    = -(MAP_SIZE // 2) * MAP_RES
        msg.info.origin.orientation.w = 1.0    # identity

        # prob is a (H,W) float array 0-1
        msg.data = (prob.reshape(-1) * 100).astype(np.int8).tolist()
        return msg
    
    def bin_grid(self):
        """Return int8 grid: -1=unknown, 0=free, 100=occ."""
        p    = self.prob_grid()
        grid = np.full_like(p, -1, dtype=np.int8)
        grid[p > OCC_THRESH]  = 100
        grid[p < FREE_THRESH] = 0
        return grid


    def clean_binary(self, bin_grid):
        occ = bin_grid == 100
        occ = binary_dilation(binary_erosion(occ, iterations=1), iterations=1)
        out = np.where(occ, 100, bin_grid)
        return out
    # -----------------------------------------------------------------------------------------------------------

    

    # ---------- Beam / Log-Odds Update ----------------------------
    def update_cellray(self, bx, by, ox, oy):
        # Ray Tracing
        bx, by = self.world_to_cell(bx, by)
        ox, oy = self.world_to_cell(ox, oy)
        cells  = bresenham(bx, by, ox, oy)

        for cx, cy in cells[:-1]:
            if self.inside(cx, cy):
                self.logodds[cy, cx] += L_FREE            # free cells

        cx, cy = cells[-1]                                # hit cell
        if self.inside(cx, cy):
            self.logodds[cy, cx] += L_OCC

            # ----------- persistence test -------------
            p = 1.0 - 1.0 / (1.0 + math.exp(-self.logodds[cy, cx]))  # P(occ)

            if p > OCC_STORE:
                if self.hit_cnt[cy, cx] < 255:                 # <- avoid wrap-around
                    self.hit_cnt[cy, cx] += 1

                if self.hit_cnt[cy, cx] == STABLE_HITS:
                    self.wall_pts.add((cx, cy))                # persist
            else:
                self.hit_cnt[cy, cx] = 0          

    # --------------------------------------------------------------------------


    # --------------------  Edge + Hough Line Detection -----------------------
    def detect_lines(self):    
        """
        Runs Probabilistic Hough directly on the cleaned binary grid.
        Returns list of ((wx1, wy1), (wx2, wy2)) line segments in metres.
        """
        # Get binary grid (occ=100, free=0, unknown=-1) 
        bin_img = self.clean_binary(self.bin_grid())          # int8
        occ     = (bin_img == 100).astype(np.uint8) * 255         # 0 / 255 image

        # Detect Edges    
        edges = cv2.Canny(occ, 50, 150, apertureSize=3)

        # Probabilistic Hough 
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.deg2rad(1),
            threshold=30,
            minLineLength=15,
            maxLineGap=4,
        )
        if lines is None:
            return []

        # Convert to world coords 
        world_lines = []
        for x1, y1, x2, y2 in lines[:, 0]:
            wx1 = (x1 - self.map_cx) * MAP_RES
            wy1 = (y1 - self.map_cy) * MAP_RES
            wx2 = (x2 - self.map_cx) * MAP_RES
            wy2 = (y2 - self.map_cy) * MAP_RES
            world_lines.append(((wx1, wy1), (wx2, wy2)))
        return world_lines
    # --------------------------------------------------------------------------

    # --------------------  Other Helper Functions -----------------------------
    def world_to_cell(self, wx, wy):
        """ Convert from world coords to integer map indices """
        cx = int(wx / MAP_RES) + self.map_cx
        cy = int(wy / MAP_RES) + self.map_cy
        return cx, cy
    

    def inside(self, cx, cy):
        """ Determine if cell indices are within map bounds """
        return 0 <= cx < MAP_SIZE and 0 <= cy < MAP_SIZE

    
    @staticmethod
    def tf_to_xyyaw(tf: TransformStamped):
        """ Extract (x,y,yaw[rad]) from geometry_msgs/TransformStamped """
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1 - 2*(q.y*q.y + q.z*q.z))
        return x, y, yaw

# --------------------  Bresenham helper--------------------------
def bresenham(x0, y0, x1, y1):
    """ Bresenham's line algorithm
        Returns a list of points between (x0,y0) and (x1,y1) 
    """
    points = []
    dx, dy = abs(x1-x0), -abs(y1-y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2*err
        if e2 >= dy: err += dy; x0 += sx
        if e2 <= dx: err += dx; y0 += sy
    return points
# --------------------------------------------------------------------------

def main():
    rclpy.init()
    rclpy.spin(Mapper())
    rclpy.shutdown()
