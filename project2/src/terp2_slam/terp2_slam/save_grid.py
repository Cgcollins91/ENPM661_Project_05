#!/usr/bin/env python3
import csv, argparse, numpy as np, rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class GridSaver(Node):
    def __init__(self, topic, outfile):
        super().__init__("grid_saver")
        self.outfile = outfile
        self.create_subscription(OccupancyGrid, topic, self.cb, 10)

    def cb(self, msg: OccupancyGrid):
        res   = msg.info.resolution
        width = msg.info.width
        height= msg.info.height
        ox    = msg.info.origin.position.x
        oy    = msg.info.origin.position.y

        data = np.asarray(msg.data, dtype=np.int8).reshape((height, width))

        tag = np.empty_like(data, dtype=object)
        tag[data == 0]   = "free"
        tag[data == 100] = "occupied"
        tag[data == -1]  = "unknown"

        with open(self.outfile, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "state"])
            for j in range(height):          # row (y)
                y = oy + (j + 0.5) * res
                for i in range(width):       # col (x)
                    x = ox + (i + 0.5) * res
                    writer.writerow([f"{x:.3f}", f"{y:.3f}", tag[j, i]])

        self.get_logger().info(f"Saved {width*height} cells → {self.outfile}")
        rclpy.shutdown()                     # one-shot

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--topic", default="/clean_map")
    parser.add_argument("-o", "--outfile", default="map_grid.csv")
    args = parser.parse_args()

    rclpy.init()
    rclpy.spin(GridSaver(args.topic, args.outfile))

if __name__ == "__main__":
    main()
