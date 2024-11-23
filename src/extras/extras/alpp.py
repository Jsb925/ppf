import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from math import atan2, sqrt, pow, sin, exp
import pandas as pd
import time
from ackermann_msgs.msg import AckermannDriveStamped


class AdaptivePurePursuit(Node):
    def __init__(self):
        super().__init__('adaptive_pure_pursuit')

        # Subscribers and Publishers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.thr_pub = self.create_publisher(
            Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        self.str_pub = self.create_publisher(
            Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.goal_pub = self.create_publisher(Marker, '/goal', 10)
        self.race_pub = self.create_publisher(MarkerArray, '/raceline', 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        # Timer for control publishing
        # timer = 1.0
        timer = 0.0001
        self.timer = self.create_timer(timer, self.publish_control_commands)

        # Parameters for Adaptive Pure Pursuit
        # self.max_speed = 0.1
        self.max_speed = 0.5
        self.min_speed = 0.3
        self.max_lookahead = 2.0
        self.min_lookahead = 1.0
        self.wheelbase = 0.33
        self.current_quaternion = [0.0, 0.0, 0.0, 1.0]
        self.lookahead_distance = self.min_lookahead
        self.beta = 0.5
        self.path = np.array([])
        self.previous_position = None
        self.previous_deviation = 0
        self.total_area = 0
        self.area_window = []
        self.window_size = 10
        self.position = None
        self.orientation = None
        self.control_velocity = 0.0015
        self.heading_angle = 0.1
        self.velocity_window = []
        self.window_velocity_size = 5
        self.smoothed_velocity = None


        # Load race line
        self.load_raceline_csv('/home/sedrica/ros2ppf_26_10/src/extras/extras/gl_track.csv')

        # tf2 setup for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)  
        self.get_logger().info("launched tf listeners")


    def load_raceline_csv(self, filename):
        self.get_logger().info("loaded raceline")

        self.path = pd.read_csv(filename)
        self.path = self.path.to_numpy().reshape(-1, 2)


        # self.path = self.path.to_numpy().T.reshape(-1, 2)

        for i in range(len(self.path)):
            self.path[i, 1] += 0.95
            self.path[i, 1] *= 1.2
            self.path[i, 0] += 1.4
            self.path[i, 0] *= 1.2

        # rotation_matrix = np.array([[0, 1], [-1, 0]])
        # self.path = np.dot(self.path, rotation_matrix.T)

    def sigmoid(self, x):
        return 1 / (1 + exp(-x))

    def update_lookahead(self, speed):
        normalized_speed = (speed - self.min_speed) / (self.max_speed - self.min_speed)
        sigmoid_value = self.sigmoid(normalized_speed * 10 - 5)

        if speed < self.min_speed:
            self.lookahead_distance = self.min_lookahead
        else:
            scaled_lookahead = self.min_lookahead + sigmoid_value * (self.max_lookahead - self.min_lookahead)
            self.lookahead_distance = min(self.max_lookahead, scaled_lookahead)

    def odom_callback(self, msg):
        self.get_logger().info("entered odom cbk")

        try:
            self.get_logger().info("try")

            # Wait for the transform from map to base_link
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            # Extract position and orientation directly from the transform
            self.position = transform.transform.translation
            self.orientation = transform.transform.rotation

            # Convert quaternion to yaw (heading angle)
            self.current_quaternion = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
            self.yaw = self.quaternion_to_yaw(self.current_quaternion)

            # Speed from odometry
            current_speed = msg.twist.twist.linear.x
            self.update_lookahead(current_speed)

            # Get the closest point and goal point on the path
            closest_point, goal_point = self.get_lookahead_point(self.position)

            if goal_point is not None:
                alpha = self.calculate_alpha(self.position, goal_point, self.yaw)
                self.heading_angle = self.calculate_heading_angle(alpha)

                area = self.calculate_deviation(self.position, closest_point)

                max_velocity_pp = self.calculate_max_velocity_pure_pursuit(self.calculate_curvature(alpha))
                min_deviation_pp = self.calculate_min_deviation_pure_pursuit(area)

                self.control_velocity = self.convex_combination(max_velocity_pp, min_deviation_pp, current_speed, area)
                self.get_logger().info("maa ki chut alpp ki")
                self.publish_control_commands()

        except tf2_ros.LookupException as e:
            self.get_logger().warn(f"Transform lookup failed: {e}")
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f"Transform extrapolation failed: {e}")

    def quaternion_to_yaw(self, quaternion):
        qx, qy, qz, qw = quaternion
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = atan2(siny_cosp, cosy_cosp)
        return yaw

    def get_lookahead_point(self, position):
        self.get_logger().info("get lookahead")

        min_dist = float('inf')
        closest_point = None
        goal_point = None

        for point in self.path:
            dist = sqrt(pow(point[0] - position.x, 2) + pow(point[1] - position.y, 2))
            if dist < min_dist:
                min_dist = dist
                closest_point = point

        closest_point_index = np.where(self.path == closest_point)

        for point in self.path:
            # self.get_logger().info("for loop1")

            dist = sqrt(pow(point[0] - position.x, 2) + pow(point[1] - position.y, 2))
            point_index = np.where(self.path == point)
            self.get_logger().info("if shivam")





            if dist > self.lookahead_distance and point_index[0][0] > closest_point_index[0][0] + 1 and point_index[0][0] < closest_point_index[0][0] + 5:
                self.get_logger().info("if yuvraj")

                goal_point = point

                # Publish the goal point as a marker
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                markerarray = MarkerArray()
                for i in range(len(self.path)):
                    # self.get_logger().info("for loop2")

                    marker1 = Marker()
                    marker1.header.frame_id = 'map'
                    marker1.header.stamp = self.get_clock().now().to_msg()
                    marker1.type = Marker.SPHERE
                    marker1.action = Marker.ADD
                    marker1.id = i
                    marker1.pose.position.x = self.path[i][0]
                    marker1.pose.position.y = self.path[i][1]
                    marker1.pose.position.z = 0.0
                    marker1.scale.x = 0.1
                    marker1.scale.y = 0.1
                    marker1.scale.z = 0.1
                    marker1.color.a = 1.0
                    marker1.color.r = 0.0
                    marker1.color.g = 1.0
                    marker1.color.b = 0.0
                    markerarray.markers.append(marker1)

                self.get_logger().info("raceline published")

                self.goal_pub.publish(marker)
                self.race_pub.publish(markerarray)
                break

        return closest_point, goal_point

    def calculate_alpha(self, position, goal_point, yaw):
        dy = goal_point[1] - position.y
        dx = goal_point[0] - position.x
        local_x = dx * np.cos(-yaw) - dy * np.sin(-yaw)
        local_y = dx * np.sin(-yaw) + dy * np.cos(-yaw)
        alpha = atan2(local_y, local_x)
        return alpha

    def calculate_heading_angle(self, alpha):
        heading_angle = atan2(2 * self.wheelbase * sin(alpha), self.lookahead_distance)
        return heading_angle

    def calculate_curvature(self, alpha):
        curvature = 2 * sin(alpha) / self.lookahead_distance
        return curvature

    def calculate_deviation(self, position, closest_point):
        deviation = sqrt(pow(closest_point[0] - position.x, 2) + pow(closest_point[1] - position.y, 2))

        if self.previous_position is not None:
            distance_traveled = sqrt(pow(position.x - self.previous_position.x, 2) + pow(position.y - self.previous_position.y, 2))
            area_increment = (deviation + self.previous_deviation) / 2 * distance_traveled
            self.area_window.append(area_increment)
            if len(self.area_window) > self.window_size:
                self.area_window.pop(0)

            self.total_area = sum(self.area_window)

        self.previous_position = position
        self.previous_deviation = deviation

        return self.total_area

    def calculate_max_velocity_pure_pursuit(self, curvature):
        max_velocity = sqrt(1 / abs(curvature)) if curvature != 0 else self.max_speed
        return min(self.max_speed, max_velocity)

    def calculate_min_deviation_pure_pursuit(self, area):
        if area > 0:
            min_deviation_velocity = self.max_speed / (1 + area)
        else:
            min_deviation_velocity = self.max_speed
        return min_deviation_velocity

    def convex_combination(self, max_velocity_pp, min_deviation_pp, current_speed, area):
        alpha = 0.5 * (current_speed - self.min_speed) / (self.max_speed - self.min_speed)
        return alpha * max_velocity_pp + (1 - alpha) * min_deviation_pp

    def moving_average_filter(self,velocity):
        self.velocity_window.append(velocity)
        if len(self.velocity_window) > self.window_velocity_size:
            self.velocity_window.pop(0)

        return sum(self.velocity_window) / len(self.velocity_window)

    def publish_control_commands(self):
        # self.get_logger().info("yaha aaya")
        self.smoothed_velocity = self.moving_average_filter(self.control_velocity)
        throttle_msg = Float32()
        throttle_msg.data = self.smoothed_velocity
        self.thr_pub.publish(throttle_msg)

        steering_msg = Float32()
        steering_msg.data = self.heading_angle
        self.str_pub.publish(steering_msg)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.frame_id = 'map'
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = self.smoothed_velocity
        drive_msg.drive.steering_angle = self.heading_angle 
        self.drive_pub.publish(drive_msg)



def main(args=None):
    rclpy.init(args=args)
    node = AdaptivePurePursuit()
    node.get_logger().info("launched node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
