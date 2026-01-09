import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # sets quality of service for nodes
from sensor_msgs.msg import Image, PointCloud2, PointField, LaserScan
from sensor_msgs_py import point_cloud2
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from std_msgs.msg import Float32MultiArray
import numpy as np
import ctypes
import math
import struct
 
# TODO: come up with a better name for this
class OnboardSonarImage(Node):
  
    # Create an ImageSubscriber class, which is a subclass of the Node class.
    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('onboard_sonar_image')

        # Cameras require quality of service to be set to best effort so they don't 
        # build up a huge backlog of frames to process and get too far behind
        qos_profile = QoSProfile(
                          reliability=QoSReliabilityPolicy.BEST_EFFORT,
                          history=QoSHistoryPolicy.KEEP_LAST,
                          depth=1
                      )


        # Create the subscriber. This subscriber will receive the data
        # from the image_mono topic.
        self.subscription = self.create_subscription(
          Image, 
          '/image_mono',
          self.listener_callback, 
          qos_profile=qos_profile
        )
        
        # Create publisher
        self.p = self.create_publisher(
            LaserScan,
            '/nekton/sonar_forward/pointcloud',
            qos_profile=qos_profile
            )
    def listener_callback(self, image):
        # Convert sonar image to point cloud
        point_cloud = self.sonar_image_to_pointcloud(image)
        self.p.publish(point_cloud)

    def sonar_image_to_pointcloud(self, sonar_image):
        sonar_array = self.image_msg_to_np(sonar_image)

        # Create empty LaserScan message
        msg = LaserScan()
        msg.header = sonar_image.header
        msg.header.frame_id = "sonar_forward"
        ranges = []
        angles = []

        # Convert sonar image to LaserScan
        for j in range(sonar_image.width - 1, 0, -1):
            intensities = sonar_array[:, j]
            intensities = np.flip(intensities)
            r = (intensities.argmax() / sonar_image.height) * 10
            theta = np.deg2rad(j / sonar_image.width * 46.1 - 23.05)

            if r > 1.2 and intensities.max() > 1:
                ranges.append(r)
            else:
                ranges.append(0.0)
            angles.append(theta)

        msg.angle_min = min(angles)
        msg.angle_max = max(angles)
        msg.angle_increment = (abs(min(angles)) + abs(max(angles))) / len(angles)
        msg.range_min = 0.0
        msg.range_max = 60.0
        msg.ranges = ranges
        return msg
    
    def image_msg_to_np(self, image_msg):
        bridge = CvBridge()
        return bridge.imgmsg_to_cv2(image_msg, desired_encoding="mono8")
    
    def polar_to_cartesian(self, r, theta):
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        return x, y
    
    
def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    ForwardSonarImageProcessor = OnboardSonarImage()
    
    # Spin the node so the callback function is called.
    rclpy.spin(ForwardSonarImageProcessor)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ForwardSonarImageProcessor.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
    
    if __name__ == '__main__':
        main()
