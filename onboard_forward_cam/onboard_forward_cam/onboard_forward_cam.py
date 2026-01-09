import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # sets quality of service for nodes
from sensor_msgs.msg import Image # Image is the message type
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
 
# TODO: come up with a better name for this
class OnboardForwardCam(Node):
  
    # Create an ImageSubscriber class, which is a subclass of the Node class.
    def __init__(self):
        
        # Initiate the Node class's constructor and give it a name
        super().__init__('onboard_forward_cam')

        # Cameras require quality of service to be set to best effort so they don't 
        # build up a huge backlog of frames to process and get too far behind
        qos_profile = QoSProfile(
                          reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                          history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                          depth=1
                      )


        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
          Image, 
          '/nekton/camera_forward/image_raw',
          self.listener_callback, 
          qos_profile=qos_profile
        )
          
        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()
   
    def listener_callback(self, data):
        # Display the message on the console
        #self.get_logger().info('Receiving video frame')
     
        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(data)
        
        # Display image
        cv2.imshow("camera", current_frame)
        
        cv2.waitKey(1)
      


def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  ForwardCamProcessor = OnboardForwardCam()
  
  # Spin the node so the callback function is called.
  rclpy.spin(ForwardCamProcessor)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  ForwardCamProcessor.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
