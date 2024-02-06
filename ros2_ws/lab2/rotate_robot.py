import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge
import numpy as np

from geometry_msgs.msg import Point, Twist

class rotate_node(Node):
    def __init__(self):
        super().__init__('rotate_robot')

        self.coordinates_x = 0
        self.coordinates_y = 0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1
        )
        self.subscription = self.create_subscription(
            Point,
            '/object_finder/coordinates',
            self.rotate_callback,
            qos_profile
        ) 
        self.publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.timer_ = self.create_timer(0.5, self.send_cmd)

    def rotate_callback(self, msg):
        self.get_logger().info('Find object at: x = (%s)' % msg.x)
        self.coordinates_x = msg.x
        self.coordinates_y = msg.y
    
    ### need to improve
    def send_cmd(self):
        cmd = Twist()
        # if ((self.msg.x-160) != 0) & ((self.msg.y-120) != 0):
        if (self.coordinates_x == 0) & (self.coordinates_y == 0):
            cmd.linear.x = 0.01
            cmd.linear.y = 0.01
            self.publisher.publish(cmd)


# def main():
#     rclpy.init()
#     rotate_node_1 = rotate_node()

#     while rclpy.ok():
#         rclpy.spin_once(rotate_node_1) # Trigger callback processing.

#     rotate_node_1.destroy_node()  
#     rclpy.shutdown()

# if __name__ == '__main__':
# 	main()