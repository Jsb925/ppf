import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray,Marker
import tf_transformations
import numpy as np

class frametf(Node):
    def __init__(self):
        super().__init__('frametf')
        

        # Subscribers and publishers
        self.path_subscription = self.create_subscription(
            Path, '/path_check', self.path_callback, 10)
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.global_path_publisher = self.create_publisher(MarkerArray, '/g2', 1)
        
        self.robot_pose = None

    def odom_callback(self, msg):
        # Update the robot pose from the odometry message
        self.robot_pose = msg.pose.pose

    def transform_pose(self, local_pose):
        if self.robot_pose is None:
            self.get_logger().warning('Robot pose is not yet available.')
            return None

        # Extract position and orientation from the robot pose
        position = self.robot_pose.position
        orientation = self.robot_pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]

        # Convert quaternion to rotation matrix
        rotation_matrix = tf_transformations.quaternion_matrix(q)[:3, :3]
        translation_vector = np.array([position.x, position.y, position.z])

        # Convert local pose to a numpy array
        local_position = np.array([local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z])
        
        # Perform the transformation
        global_position = np.dot(rotation_matrix, local_position) + translation_vector
        
        # Create a new PoseStamped message in the global frame
        global_pose = PoseStamped()
        global_pose.header.stamp = local_pose.header.stamp
        global_pose.header.frame_id = 'world'
        global_pose.pose.position.x = global_position[0]
        global_pose.pose.position.y = global_position[1]
        global_pose.pose.position.z = global_position[2]
        global_pose.pose.orientation = local_pose.pose.orientation  # Keep orientation as is

        return global_pose

    def path_callback(self, msg):
        global_path = MarkerArray()
        i=0
        test_path=Path()
        test_path.header.frame_id='world'
        test_path.header.stamp = self.get_clock().now().to_msg()
        # ln=[[1.0,1.0],[2.0,3.0],[3.0,5.0]]
        # for i in range(len(ln)):
        #     pose=PoseStamped()
        #     pose.pose.position.x=ln[i][0]
        #     pose.pose.position.y=ln[i][1]
        #     test_path.poses.append(pose)

        # Iterate over each pose in the path
        for pose in msg.poses :
            global_pose = self.transform_pose(pose)
            print(global_pose)
            if global_pose is not None:
                marker = Marker()
                marker.header.frame_id='world'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose = global_pose.pose
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                global_path.markers.append(marker)
                i+=1
        
        # Publish the transformed global path
        print("huisdrfgbn")
        self.global_path_publisher.publish(global_path)
        


        # for pose in msg.poses:
        #     global_pose = self.transform_pose(pose)
        #     if global_pose is not None:
        #         global_path.poses.append(global_pose)

        # # Publish the transformed global path
        # self.global_path_publisher.publish(global_path)

def main(args=None):
    rclpy.init(args=args)
    path_transformer = frametf()
    rclpy.spin(path_transformer)
    path_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
