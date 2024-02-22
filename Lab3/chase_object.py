import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge
import numpy as np

from geometry_msgs.msg import Point, Twist

class chase_object(Node):
    def __init__(self):
        super().__init__('chase_object')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1
        )

        self.subscription = self.create_subscription(
            Point,
            '/get_object_range/coordinates',
            self.get_coordinates,
            qos_profile
        )        

        self.publisher = self.create_publisher(Twist,'/cmd_vel',5)
        self.angle = 0
        self.dist = 0.4
        self.safe_dist = 0.4
        self.Kp_v = 1
        self.Kp_a = 0.1
        self.timer_1 = self.create_timer(0.5, self.angle_position_PID)
        
    def get_coordinates(self,msg):
        self.angle = msg.x
        self.dist = msg.y

    def angle_position_PID(self):
        cmd = Twist()
        if self.angle == 9999.0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        elif (np.abs(self.angle) <= 8):
            cmd.angular.z = 0.0     # threshold of tracking area

        else:
            cmd.angular.z = self.limit(-self.Kp_a*self.angle, -0.5, 0.5)   # P control of angle 

        if self.dist <= 3:
            error = self.dist - self.safe_dist
	    if np.abs(error) <= 0.05: # allow robot to stay in a approximate range 
	    	cmd.linear.x = 0.0		
	    else:
		cmd.linear.x = self.limit(self.Kp_v*error,-0.1, 0.1)
		    
        self.publisher.publish(cmd)
        
    # Limit function
    def limit(self, val, min, max):
        if val >= max:
            val = max
        elif val <= min:
            val = min
        return val

def main():
    rclpy.init()
    chase_object1 = chase_object()

    while rclpy.ok():
        rclpy.spin_once(chase_object1) # Trigger callback processing.

    chase_object1.destroy_node()  
    rclpy.shutdown()

if __name__ == '__main__':
	main()
