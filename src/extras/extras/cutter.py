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
            '/gamma1',
            10
        )
        self.img_pub2 = self.create_publisher(
            Image, 
            '/gamma2',
            10
        )
        # Map size and scale
        self.map_scale = 100  # Scale for converting coordinates to pixels (pixels per meter)
        
        # Initialize a blank map
        self.map_img_orig = cv2.imread('/home/sedrica/ros2ppf_26_10/src/extras/track/my_map_new.png',cv2.IMREAD_GRAYSCALE)
        self.map_img = self.map_img_orig.copy()
        self.mapsize_r = self.map_img.shape[0]
        self.mapsize_c = self.map_img.shape[1]
        
        # Offset to center the map on (0,0)
        self.offset_r = self.mapsize_r // 2
        self.offset_c = self.mapsize_c // 2

        self.get_logger().info("Odom Visualizer Node started.")
    
    def odom_callback(self, msg):
        # Extract position and orientation from Odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        print("x: ",x)
        print("y: ",y)
        orientation = msg.pose.pose.orientation

        # Convert quaternion to yaw angle
        self.yaw = self.quaternion_to_yaw(orientation)

        # Convert world coordinates to map pixel coordinates
        pixel_r = int(-x * self.map_scale) + self.offset_r +352-196
        pixel_c = int(-y * self.map_scale) + self.offset_c
        print("pixel_r: ",pixel_r)
        print("pixel_c: ",pixel_c)

        # Draw the position on the map
        self.draw_position(pixel_r, pixel_c, self.yaw)
        final=self.extract_yaw_roi(self.map_img,np.array([[pixel_r,pixel_c]],dtype=np.float32),np.rad2deg(self.yaw),self.map_scale*5,50)
   
    def dot(self, matrix, pointr,pointc,m,n=0):
        matrix[int(pointr-m):int(pointr+m),int(pointc-m):int(pointc+m)]=n
    def extract_yaw_roi(self,matrix,vect,yaw,input_scale,target_scale):
        '''
        matrix: np array
        vect: 1x2 np array
        self.yaw: 0 to 360deg
        input_scale: 1x1 np array
        target_scale: 1x1 np array
        '''
        ###########when rotate around a point, the point is invariant


        yaw=(-90-yaw)
        # matrix = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
        M = cv2.getRotationMatrix2D((vect[0,1], vect[0,0]), yaw, 1.0)
        # M = cv2.getRotationMatrix2D((vect[0,0], vect[0,1]), yaw, 1.0)
        self.dot(matrix,vect[0,0],vect[0,1],5)
        img = cv2.warpAffine(matrix, M, (matrix.shape[1],matrix.shape[0]))
        pts = np.array([[-(input_scale/2-1), 0],
                        [-(input_scale/2-1), input_scale-1],
                        [input_scale/2-1, input_scale-1],
                        [input_scale/2-1, 0]],dtype=np.float32)
        pts_shifted = pts + vect
        pts_shifted_0_1_swapped = np.array([pts_shifted[:,1],pts_shifted[:,0]]).T
        for i in pts_shifted_0_1_swapped:
            self.dot(img,i[1],i[0],20)
        self.pub_img2(img)               

        rts=img.copy()
        # for i in pts_shifted:
        #     self.dot(rts,i[1],i[0],20,0)

       
        roi =rts[int(pts_shifted[0,0]):int(pts_shifted[2,0]),int(pts_shifted[0,1]):int(pts_shifted[2,1])]
        deg90=cv2.getRotationMatrix2D((roi.shape[0]//2,roi.shape[0]//2), 90, 1.0)
        roi=cv2.warpAffine(roi,deg90,roi.shape)
        self.pub_img(roi)

        M = cv2.getRotationMatrix2D((target_scale//2,target_scale//2), 90, 1.0)
        roi_turned = cv2.warpAffine(roi, M, roi.shape)
        

        print(pts_shifted_0_1_swapped)
        return roi_turned
    def quaternion_to_yaw(self, orientation):
        """
        Convert quaternion orientation to self.yaw angle.
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
        cv2.circle(self.map_img, (y, x), 5, (0, 0, 255), -1)

        # Draw the heading as a line
        line_length = 20  # Length of the heading line
        end_x = int(x + line_length * math.cos(self.yaw))
        end_y = int(y + line_length * math.sin(self.yaw))
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
    
    def pub_img2(self,cv2_imag_gray):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom_visualizer'
        msg.height = cv2_imag_gray.shape[0]
        msg.width = cv2_imag_gray.shape[1]
        msg.encoding = 'mono8'
        msg.is_bigendian = 0
        msg.step = msg.width
        msg.data = cv2_imag_gray.tobytes()
        self.img_pub2.publish(msg)
    

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
