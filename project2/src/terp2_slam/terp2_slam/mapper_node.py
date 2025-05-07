#!/usr/bin/env python3
import rclpy, math, numpy as np
from rclpy.node           import Node
from sensor_msgs.msg      import LaserScan
from nav_msgs.msg         import OccupancyGrid, Odometry
from tf2_ros              import TransformBroadcaster, TransformListener, Buffer, StaticTransformBroadcaster   
from geometry_msgs.msg    import TransformStamped
from rclpy.qos            import qos_profile_sensor_data

MAP_RES   = 0.05          # metres / cell
MAP_SIZE  = 600           # 400 × 400 → 20 m × 20 m
L0        = 0.0           # log-odds prior
L_OCC     = 2.0
L_FREE    = -1.0

class Mapper(Node):
    def __init__(self):
        super().__init__("simple_mapper")

        # ---------- map storage ----------
        self.logodds = np.full((MAP_SIZE, MAP_SIZE), L0, dtype=np.float32)
        self.map_cx  = MAP_SIZE // 2        # world (0,0) at cell centre
        self.map_cy  = MAP_SIZE // 2

        # ---------- publishers / TF ----------
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 2)

        # TF helpers
        self.tf_buf  = Buffer()
        self.tf_lstn = TransformListener(self.tf_buf, self)      # listener
        self.tf_br   = TransformBroadcaster(self)                # dynamic broadcaster
        self.static_br = StaticTransformBroadcaster(self)        # once-off broadcaster

        # make the ‘map’ frame exist immediately -------------
        t = TransformStamped()
        now = self.get_clock().now().to_msg()
        t.header.stamp    = now
        t.header.frame_id = "map"
        t.child_frame_id  = "odom"            # <-- keep the same name you use elsewhere
        t.transform.rotation.w = 1.0          # identity quaternion
        self.static_br.sendTransform(t)

        # Laser-scan subscription -----------------------------
        self.create_subscription(
            LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data
        )

        # periodic (dynamic) map→odom update ------------------
        self.tf_timer = self.create_timer(0.5, self.pub_map_tf)

    # ---------- callbacks ----------
    def scan_cb(self, scan: LaserScan):
        try:
            tf = self.tf_buf.lookup_transform(
                            "odom", scan.header.frame_id,
                            rclpy.time.Time(), # 0 = latest transform
                            timeout=rclpy.duration.Duration(seconds=0.1))       
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        x, y, yaw = self.tf_to_xyyaw(tf)

        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment,
                           dtype=np.float32)
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        valid  = np.isfinite(ranges)
        angles, ranges = angles[valid], ranges[valid]

        # Iterate over all beam endpoints -------------------------------
        ex = x + ranges * np.cos(angles + yaw)
        ey = y + ranges * np.sin(angles + yaw)

        for bx, by, ox, oy in zip(
                np.full_like(ex, x), np.full_like(ey, y), ex, ey):
            cbx, cby = self.world_to_cell(bx, by)
            cox, coy = self.world_to_cell(ox, oy)

            if not (self.inside(cbx, cby) or self.inside(cox, coy)):
                continue            # whole ray off-map → skip

            self.update_cellray(bx, by, ox, oy)

        self.publish_map(scan.header.stamp)

    # ---------- helpers ----------
    def world_to_cell(self, wx, wy):
        cx = int(wx / MAP_RES) + self.map_cx
        cy = int(wy / MAP_RES) + self.map_cy
        return cx, cy
    
    def inside(self, cx, cy):
        return 0 <= cx < MAP_SIZE and 0 <= cy < MAP_SIZE

    def update_cellray(self, bx, by, ox, oy):
        bx, by = self.world_to_cell(bx, by)
        ox, oy = self.world_to_cell(ox, oy)
        cells = bresenham(bx, by, ox, oy)
        for c in cells[:-1]:
            self.logodds[c[1], c[0]] += L_FREE
        self.logodds[cells[-1][1], cells[-1][0]] += L_OCC

    def publish_map(self, stamp):
        grid = OccupancyGrid()
        grid.header.stamp    = stamp
        grid.header.frame_id = "map"
        grid.info.resolution = MAP_RES
        grid.info.width      = MAP_SIZE
        grid.info.height     = MAP_SIZE
        grid.info.origin.position.x = -(MAP_SIZE//2)*MAP_RES
        grid.info.origin.position.y = -(MAP_SIZE//2)*MAP_RES
        grid.data = (100 * (1 - 1/(1+np.exp(self.logodds))).flatten()).astype(np.int8).tolist()
        self.map_pub.publish(grid)

    def pub_map_tf(self):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id  = "odom"
        # identity: no pose correction yet
        self.tf_br.sendTransform(t)        # <-- use broadcaster

    @staticmethod
    def tf_to_xyyaw(tf: TransformStamped):
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1 - 2*(q.y*q.y + q.z*q.z))
        return x, y, yaw

def bresenham(x0, y0, x1, y1):
    """Classic integer line algorithm → list of (x,y) tuples."""
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

def main():
    rclpy.init()
    rclpy.spin(Mapper())
    rclpy.shutdown()
