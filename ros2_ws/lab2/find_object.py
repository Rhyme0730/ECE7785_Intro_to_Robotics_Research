import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge
import numpy as np

from geometry_msgs.msg import Point
from rotate_robot import rotate_node

class object_finder(Node):
    def __init__(self):
        super().__init__('object_finder')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            depth=1
        )
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self._image_callback,
            qos_profile
        )

        self.publisher1 = self.create_publisher(CompressedImage,'/object_finder/compressed',qos_profile)
        self.publisher2 = self.create_publisher(Point,'/object_finder/coordinates',qos_profile)
    def _image_callback(self, CompressedImage):	
        self.find_object(CompressedImage)
        
    def find_object(self, CompressedImage):
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage,"bgr8")
        self.gray = cv2.medianBlur(cv2.cvtColor(self._imgBGR, cv2.COLOR_BGR2GRAY),5)
        self.circles = cv2.HoughCircles(self.gray,cv2.HOUGH_GRADIENT,1,300,
                            param1=70,param2=60,minRadius=0,maxRadius=0)
        if self.circles is not None:
            self.circles = np.uint16(np.around(self.circles))
            for i in self.circles[0,:]:
                cv2.circle(self._imgBGR,(i[0],i[1]),i[2],(0,255,0),2)
                cv2.circle(self._imgBGR,(i[0],i[1]),2,(0,0,255),3)
                self.get_logger().info('Find object at: x = (%s)' % i[0])

                self.msg = Point()
                self.msg.x = float(i[0])
                self.msg.y = float(i[1])
                self.publisher2.publish(self.msg)
        self.publisher1.publish(CvBridge().cv2_to_compressed_imgmsg(self._imgBGR))

def main():
    rclpy.init()
    object_finder_1 = object_finder()
    rotate_node_1 = rotate_node()

    while rclpy.ok():
        rclpy.spin_once(object_finder_1) # Trigger callback processing.
        rclpy.spin_once(rotate_node_1)
    object_finder_1.destroy_node()
    rotate_node_1.destroy_node()    
    rclpy.shutdown()

if __name__ == '__main__':
	main()