#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose


'''
Warning
any incidence of seemingly unexplained integers/doubles (eg. 49, 24, 5 ,2.5 etc)
are due to the size of target matrix
and must be changed with target matrix size
'''


class convertor(Node):
    def __init__(self):
        super().__init__('laser_to_grid')

        # Parameters
        self.grid_resolution = self.declare_parameter(
            "grid_resolution", 0.1).value  # Grid cell size in meters
        self.grid_width = self.declare_parameter("grid_width", 50).value  # Number of cells along x-axis
        self.grid_height = self.declare_parameter("grid_height", 50).value  # Number of cells along y-axis

        # MAP ORIGIN - 0,0 is center of grid, not corner
        self.grid_origin_x = self.declare_parameter(
            "grid_origin_x", 0).value  # Map origin x (in meters)
        self.grid_origin_y = self.declare_parameter(
            "grid_origin_y", -2.5).value  # Map origin y (in meters)

        # Prepare the occupancy grid
        self.occupancy_grid = OccupancyGrid()
        #TODO ####################################### fix frames
        self.occupancy_grid.header.frame_id = "lidar"
        self.occupancy_grid.info.resolution = self.grid_resolution
        self.occupancy_grid.info.width = self.grid_width
        self.occupancy_grid.info.height = self.grid_height
        self.occupancy_grid.info.origin = Pose()
        self.occupancy_grid.info.origin.position.x =float(self.grid_origin_x)
        self.occupancy_grid.info.origin.position.y =float(self.grid_origin_y)
        self.occupancy_grid.info.origin.position.z = 0.0
        self.occupancy_grid.info.origin.orientation.w = 1.0

        # Initialize the grid with unknown (-1) values
        self.grid = np.full((self.grid_width, self.grid_height), -1)

        # Subscriber to LaserScan messages
        self.scan_sub = self.create_subscription(
            LaserScan, '/autodrive/f1tenth_1/lidar', self.scan_callback, 10)

        # Publisher for OccupancyGrid messages
        self.grid_pub = self.create_publisher(OccupancyGrid, '/map2', 10)

    def scan_callback(self, scan_msg):
        """Callback for processing LaserScan messages and updating the occupancy grid."""
        # Reset the grid for every scan
        self.grid.fill(-1)

        # Iterate through all laser scan ranges
        angle = scan_msg.angle_min
        for i, r in enumerate(scan_msg.ranges):
            if scan_msg.range_min < r < scan_msg.range_max:
                # Calculate the position of the laser hit in real-world coordinates

                # Map size-specific values - play with these first
                hit_x = r * math.sin(angle) + 2.5
                hit_y = r * math.cos(angle) - 2.5

                # Convert to grid coordinates
                grid_x = int((hit_x - self.grid_origin_x) / self.grid_resolution)
                grid_y = int((hit_y - self.grid_origin_y) / self.grid_resolution)

                # Make sure the point is within the bounds of the grid
                if 1 <= grid_x < self.grid_width - 1 and 1 <= grid_y < self.grid_height - 1:
                    self.grid[grid_x, grid_y] = 100  # Mark as occupied
                    self.grid[grid_x + 1, grid_y] = 100
                    self.grid[grid_x - 1, grid_y] = 100
                    self.grid[grid_x, grid_y + 1] = 100
                    self.grid[grid_x, grid_y - 1] = 100

            # Increment the angle for the next scan
            angle += scan_msg.angle_increment

        # Flatten the grid and publish it as an OccupancyGrid message
        self.occupancy_grid.data = self.grid.flatten().tolist()
        self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        self.grid_pub.publish(self.occupancy_grid)


def main(args=None):
    rclpy.init(args=args)
    node = convertor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
