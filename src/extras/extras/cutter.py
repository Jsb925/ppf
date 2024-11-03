import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import numpy as np
import cv2
import math

class OdomVisualizer(Node):
    def __init__(self):
        super().__init__('odom_visualizer')
        
        # Subscription to the Odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.img_pub = self.create_publisher(
            Image, 
            'image',
            10
        )
        # Map size and scale
        self.map_scale = 10  # Scale for converting coordinates to pixels (pixels per meter)
        
        # Initialize a blank map
        self.map_img_orig = cv2.imread('/home/sedrica/ros2ppf_26_10/src/extras/track/my_map_new.png',cv2.IMREAD_GRAYSCALE)
        self.map_img = self.map_img_orig.copy()
        self.mapsize_r = self.map_img.shape[0]
        self.mapsize_h = self.map_img.shape[1]
        
        # Offset to center the map on (0,0)
        self.offset_r = self.mapsize_r // 2
        self.offset_c = self.mapsize_c // 2

        self.get_logger().info("Odom Visualizer Node started.")
    
    def odom_callback(self, msg):
        # Extract position and orientation from Odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation

        # Convert quaternion to yaw angle
        yaw = self.quaternion_to_yaw(orientation)

        # Convert world coordinates to map pixel coordinates
        pixel_x = int(x * self.mapscale_r) + self.offset_r
        pixel_y = int(y * self.mapscale_c) + self.offset_c

        # Draw the position on the map
        self.draw_position(pixel_x, pixel_y, yaw)
        
        # Display the updated map
        cv2.imshow('Odometry Map', self.map_img)
        cv2.waitKey(1)
    
    def quaternion_to_yaw(self, orientation):
        """
        Convert quaternion orientation to yaw angle.
        """
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        return math.atan2(siny_cosp, cosy_cosp)

    def draw_position(self, x, y, yaw):
        """
        Draw the robot position and heading on the map.
        """
        # Clear the map
        self.map_img = self.map_img_orig.copy()
        # Draw the position as a circle
        cv2.circle(self.map_img, (x, y), 5, (0, 0, 255), -1)

        # Draw the heading as a line
        line_length = 20  # Length of the heading line
        end_x = int(x + line_length * math.cos(yaw))
        end_y = int(y + line_length * math.sin(yaw))
        cv2.line(self.map_img, (x, y), (end_x, end_y), (0, 255, 0), 2)

    def pub_img(self,cv2_imag_gray):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom_visualizer'
        msg.height = cv2_imag_gray.shape[0]
        msg.width = cv2_imag_gray.shape[1]
        msg.encoding = 'mono8'
        msg.is_bigendian = 0
        msg.step = msg.width
        msg.data = cv2_imag_gray.tobytes()
        self.img_pub.publish(msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = OdomVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Save the map when the node is interrupted
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
