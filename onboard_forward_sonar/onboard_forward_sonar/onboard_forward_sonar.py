import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # sets quality of service for nodes
from sensor_msgs.msg import PointCloud2, PointField # PointCloud2 is the message type
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from std_msgs.msg import Float32MultiArray
import numpy as np
import ctypes
import math
import struct

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)
 
# TODO: come up with a better name for this
class OnboardForwardSonar(Node):
  
    # Create an ImageSubscriber class, which is a subclass of the Node class.
    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('onboard_forward_sonar')

        # Cameras require quality of service to be set to best effort so they don't 
        # build up a huge backlog of frames to process and get too far behind
        qos_profile = QoSProfile(
                          reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                          history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                          depth=1
                      )


        # Create the subscriber. This subscriber will receive the point cloud
        # from the forward_sonar topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
          PointCloud2, 
          '/nekton/sonar_forward',
          self.listener_callback, 
          qos_profile=qos_profile
        )
        
        
        # Create publisher
        self.p = self.create_publisher(
            Float32MultiArray,
            '/nekton/sonar_forward/sonar_points',
            qos_profile=qos_profile
            )
   
    def _get_struct_fmt(self, is_bigendian, fields, field_names=None):
        fmt = '>' if is_bigendian else '<'
        offset = 0
        for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
            if offset < field.offset:
                fmt += 'x' * (field.offset - offset)
                offset = field.offset
            if field.datatype not in _DATATYPES:
                print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
            else:
                datatype_fmt, datatype_length = _DATATYPES[field.datatype]
                fmt    += field.count * datatype_fmt
                offset += field.count * datatype_length
        return fmt

    def read_points(self, cloud, field_names=None, skip_nans = True):
        fmt = self._get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
        unpack_from = struct.Struct(fmt).unpack_from
        width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
        my_msg = Float32MultiArray()
        if skip_nans:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        for v in p:
                            my_msg.data.append(v)
                    offset += point_step
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    for v in p:
                        my_msg.data.append(v)
                    offset += point_step
        return my_msg

    def listener_callback(self, cloud):
        msg = self.read_points(cloud, field_names = ['x', 'y', 'z', 'intensity'])
        self.p.publish(msg)

def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    ForwardSonarProcessor = OnboardForwardSonar()
    
    # Spin the node so the callback function is called.
    rclpy.spin(ForwardSonarProcessor)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ForwardSonarProcessor.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
    
    if __name__ == '__main__':
        main()
