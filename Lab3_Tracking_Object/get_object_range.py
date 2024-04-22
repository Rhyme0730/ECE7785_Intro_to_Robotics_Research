import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge
import numpy as np

from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan


class get_object_range(Node):
    def __init__(self):
        super().__init__('get_object_range')

        self.angle = 0.0 # may need change

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1
        )
        self.subscription = self.create_subscription(
            Point,
            '/detect_object/coordinates',
            self.get_angle,
            qos_profile
        ) 
        self.subscription_LIDAR = self.create_subscription(
            LaserScan,
            '/scan',
            self.get_distance,
            qos_profile
        )
        self.publisher = self.create_publisher(Point,'/get_object_range/coordinates',qos_profile)

    def get_distance(self,msg):
        self.unit_angle = msg.angle_increment # get the unit angle value of Lidar
            
        ### get the radians in the Lidar frame
        if self.angle >= 0:
            radians = (self.angle/180)*np.pi
        else:
            radians = (self.angle/180+2)*np.pi
        index = min(int(radians/self.unit_angle),len(msg.ranges)-1)

        ### if not detected 
        if self.angle == 9999.0: 
            self.distance = 0.4 # safe distance
        else:
            num = 0
            dist = 0
            r = 2 
            for i in range (r*2):
                idx = index-r+i
                if index < 0:
                    idx = len(msg.ranges)+idx
                if idx >= len(msg.ranges):
                    idx -= len(msg.ranges)
                if np.isnan(msg.ranges[idx]) or msg.ranges[idx]>=1.8:
                    continue
                num += 1
                dist += msg.ranges[idx]
            if num == 0:
                return
                
            self.distance = dist/num

            # self.distance = msg.ranges[index]
            self.get_logger().info(f'Received LiDAR scan with {self.distance}')
            # self.get_logger().info(f'i-1:Received LiDAR scan with {msg.ranges[index-1]}')
            
        # publish msg
        self.msg1 = Point()
        self.msg1.x = self.angle
        self.msg1.y = self.distance
        self.publisher.publish(self.msg1)

    def get_angle(self, msg):
        self.angle = msg.x


def main():
    rclpy.init()
    get_object_range1 = get_object_range()

    while rclpy.ok():
        rclpy.spin_once(get_object_range1) # Trigger callback processing.

    get_object_range1.destroy_node()  
    rclpy.shutdown()

if __name__ == '__main__':
	main()
