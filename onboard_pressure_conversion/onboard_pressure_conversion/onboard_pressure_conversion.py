import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # sets quality of service for nodes
from std_msgs.msg import Float32
from sensor_msgs.msg import FluidPressure
 
class OnboardPressureConversion(Node):
  
    def __init__(self):
        
        # Initiate the Node class's constructor and give it a name
        super().__init__('onboard_pressure_conversion')
        self.publisher_ = self.create_publisher(Float32, '/nekton/depth', 10) ### 

        # Create the subscriber. This subscriber will receive a FluidPressure
        # from the /nekton/pressure topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
          FluidPressure,
          '/nekton/pressure',
          self.listener_callback, 
          1
        )
          
    def listener_callback(self, data):
        # Display the message on the console
        self.get_logger().info('Receiving FluidPressure')

        #Convert pressure in Pascals to depth
        depth = data.fluid_pressure / (1025 * 9.8)


        #Publish depth to depth topic
        msg = Float32()
        msg.data = depth
        self.publisher_.publish(msg)
      


def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  OnboardPressureConvert = OnboardPressureConversion() ### !
  
  # Spin the node so the callback function is called.
  rclpy.spin(OnboardPressureConvert)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  OnboardPressureConvert.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
