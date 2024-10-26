#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from math import atan2, sqrt, exp
import tf_transformations as tf
import pandas as pd
import time


'''
Warning
any incidence of seemingly unexplained integers (eg. 49, 24 etc)
are due to the size of target matrix
and must be changed with target matrix size
'''

class direction(Node):
    def __init__(self):
        super().__init__('direction')

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.goal_pub = self.create_publisher(Marker, '/goal', 1)
        self.race_pub = self.create_publisher(MarkerArray, '/raceline', 1)
        self.arrow_pub = self.create_publisher(Marker, '/target_arrow', 1)

        self.max_speed = 6.0
        self.min_speed = 1.0
        self.max_lookahead = 2.0
        self.min_lookahead = 1.0
        self.wheelbase = 0.33
        self.lookahead_distance = self.min_lookahead
        self.beta = 0.5

        self.path = np.array([])  
        self.previous_position = None
        self.previous_deviation = 0
        self.total_area = 0
        self.area_window = []
        self.window_size = 10

        ##TODO fix this link
        self.load_raceline_csv('/home/sedrica/ros2ppf/src/extras/track/gl_track.csv')

        # Set up the loop rate
        self.rate = self.create_rate(10)  # 10 Hz

    def load_raceline_csv(self, filename):
        self.path = pd.read_csv(filename)
        self.path = np.array([self.path]).reshape(-1, 2)

        for i in range(len(self.path)):
            self.path[i, 1] += 0.5
            self.path[i, 0] -= 2.8

        rotation_matrix = np.array([[0, 1], [-1, 0]])
        self.path = np.dot(self.path, rotation_matrix.T)

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

    def radian_correction(self, val):
        out = val
        if (val - np.pi / 2) <= 0 and (val - np.pi / 2) >= (-np.pi):
            out = -val
        elif (val - np.pi / 2) > 0 and (val - np.pi / 2) < np.pi:
            out = 2 * np.pi - val
        return out + np.pi / 2
    
    def radian_correction2(self, val):
        out = val
        if val <= 0 and val >= (-np.pi):
            out = 1.5 * np.pi + val
        elif val > 0 and val <= np.pi / 2:
            out = 1.5 * np.pi + val
        elif val > np.pi / 2 and val <= np.pi:
            out = val - np.pi / 2
        return out

    def odom_callback(self, msg):
        print("triggered")
        position = msg.pose.pose.position
        orientation_n = msg.pose.pose.orientation
        heading = self.radian_correction(self.quaternion_to_yaw([orientation_n.x, orientation_n.y, orientation_n.z, orientation_n.w]) + np.pi / 2)
        current_speed = sqrt(msg.twist.twist.linear.x ** 2 + msg.twist.twist.linear.y ** 2)

        closest_point, goal_point = self.get_lookahead_point(position)

        goal_angle = self.radian_correction(np.arctan2(goal_point[0], goal_point[1]))
        car_angle = self.radian_correction(np.arctan2(position.x, position.y))
        goal_car_angle = self.radian_correction2(np.arctan2((goal_point[0] - position.x), goal_point[1] - position.y))
        target_angle = goal_car_angle - heading
        if target_angle < 0: 
            target_angle += 2 * np.pi

        distance_goal_car = sqrt((position.y - goal_point[1])**2 + (position.x - goal_point[0])**2)
        self.distance = distance_goal_car
        self.target_yaw = target_angle
        self.publish_target_arrow(target_angle)

        print("mew")
        self.get_logger().info(f"distance_goal_car: {distance_goal_car}")
        self.get_logger().info(f"target_angle: {target_angle}")
        self.get_logger().info(f"heading: {heading}")
        self.get_logger().info(f"goal_car_angle: {goal_car_angle * 180 / np.pi}")
        self.get_logger().info(f"goal_angle: {goal_angle}")
        self.get_logger().info(f"car_angle: {car_angle}")

    def quaternion_to_yaw(self, quaternion):
        qx, qy, qz, qw = quaternion
        roll = atan2(2.0 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz)
        return roll

    def euler_to_quaternion(self,yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
    def publish_target_arrow(self, yaw):
        arrow_marker = Marker()
        arrow_marker.header.frame_id = 'f1tenth_1'
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD        
        arrow_marker.pose.position.x = 0.0
        arrow_marker.pose.position.y = 0.0
        arrow_marker.pose.position.z = 0.0
        print("pspspss")
       # quaternion = self.euler_to_quaternion(0,0,-self.target_yaw)
        q= Quaternion()
       #q.x=quaternion[0]
        #q.y=quaternion[1]
        #q.z=quaternion[2]
        #q.w=quaternion[3]
        q.x=-self.target_yaw

        arrow_marker.pose.orientation=q


        arrow_marker.scale.x = self.distance  # Shaft diameter
        arrow_marker.scale.y = 0.2  # Head diameter
        arrow_marker.scale.z = 0.2  # Head length
        arrow_marker.color.a = 1.0
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 1.0  # Blue color for the arrow
        
        self.arrow_pub.publish(arrow_marker)
        print(arrow_marker)
        print("mrrrr")

    def get_lookahead_point(self, position):
        min_dist = float('inf')
        closest_point = None
        goal_point = None

        for point in self.path:
            dist = sqrt(pow(point[0] - position.x, 2) + pow(point[1] - position.y, 2))
            if dist < min_dist:
                min_dist = dist
                closest_point = point

        closest_point_index = np.where(self.path == closest_point)[0][0]

        for point in self.path:
            dist = sqrt(pow(point[0] - position.x, 2) + pow(point[1] - position.y, 2))
            point_index = np.where(self.path == point)[0][0]
            if dist > self.lookahead_distance and point_index > closest_point_index + 2:
                goal_point = point
                self.publish_goal_marker(point)
                break
        
        return closest_point, goal_point

    def publish_goal_marker(self, goal_point):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = goal_point[0]
        marker.pose.position.y = goal_point[1]
        marker.pose.position.z = 0.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        print(marker)
        print(self.path)
        marker_array = MarkerArray()
        for i in range(len(self.path)):
            marker1 = Marker()
            marker1.header.frame_id = 'world'
            marker1.header.stamp = self.get_clock().now().to_msg()
            marker1.type = Marker.SPHERE
            marker1.action = Marker.ADD
            marker1.id = i
            marker1.pose.position.x=self.path[i][0]
            marker1.pose.position.y=self.path[i][1]
            marker1.pose.position.z=0.0
            marker1.scale.x = 0.1
            marker1.scale.y = 0.1
            marker1.scale.z = 0.1
            marker1.color.a = 1.0
            marker1.color.r = 0.0
            marker1.color.g = 1.0
            marker1.color.b = 0.0
            marker_array.markers.append(marker1)

        self.race_pub.publish(marker_array)
        self.goal_pub.publish(marker)
        print("smeee")
    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = direction()
    rate = node.create_rate(0.1)

    try:
        rclpy.spin(node)
        rate.sleep()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
