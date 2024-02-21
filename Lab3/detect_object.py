import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge
import numpy as np

from geometry_msgs.msg import Point

class detect_object(Node):
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

        self.publisher1 = self.create_publisher(CompressedImage,'/detect_object/compressed',qos_profile) # publish camera image
        self.publisher2 = self.create_publisher(Point,'/detect_object/coordinates',qos_profile) # publish angles 

    def _image_callback(self, CompressedImage):	
        self.find_object_Hough(CompressedImage)
        # self.find_object_Contour(CompressedImage)
        
    def find_object_Hough(self, CompressedImage):
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage,"bgr8")
        self.gray = cv2.medianBlur(cv2.cvtColor(self._imgBGR, cv2.COLOR_BGR2GRAY),5)
        self.circles = cv2.HoughCircles(self.gray,cv2.HOUGH_GRADIENT,1,300,
                            param1=70,param2=60,minRadius=0,maxRadius=0)
        if self.circles is not None:
            self.circles = np.uint16(np.around(self.circles))
            for i in self.circles[0,:]:
                cv2.circle(self._imgBGR,(i[0],i[1]),i[2],(0,255,0),2)
                cv2.circle(self._imgBGR,(i[0],i[1]),2,(0,0,255),3)
                # self.get_logger().info('Find object at: x = (%s)' % i[0])
                self.msg = Point()
                self.msg.x = ((i[0]-160)/320)*62.2  # angle of object
                self.get_logger().info('object angle: theta = (%s)' % self.msg.x)

        elif self.circles is None:
            self.msg = Point()
            self.msg.x = 9999.0

        self.publisher1.publish(CvBridge().cv2_to_compressed_imgmsg(self._imgBGR))
        self.publisher2.publish(self.msg)

    # def find_object_Contour(self, CompressedImage):
    #     self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage,"bgr8")
    #     self.hsv_image = cv2.cvtColor(self._imgBGR, cv2.COLOR_BGR2HSV)

    #     lower_orange = np.array([5, 50, 50])
    #     upper_orange = np.array([15, 255, 255])

    #     mask = cv2.inRange(self.hsv_image, lower_orange, upper_orange)

    #     contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    #     if contours:
    #         for contour in contours:
    #             x, y, w, h = cv2.boundingRect(contour)

    #             cv2.rectangle(self._imgBGR, (x, y), (x+w, y+h), (0, 255, 0), 2)

    #             center_x = x + w // 2
    #             center_y = y + h // 2
    #             cv2.circle(self._imgBGR, (center_x, center_y), 5, (255, 0, 0), -1)
    #             self.get_logger().info(f'Coordinates{center_x, center_y}')

    #     else:
    #         self.get_logger().info('No contours found')
    #         self.msg = Point()
    #         self.msg.x = 9999.0

    #     self.publisher1.publish(CvBridge().cv2_to_compressed_imgmsg(self._imgBGR))
    #     self.publisher2.publish(self.msg)

def main():
    rclpy.init()
    detect_object_1 = detect_object()

    while rclpy.ok():
        rclpy.spin_once(detect_object_1) # Trigger callback processing.
    detect_object_1.destroy_node() 
    rclpy.shutdown()

if __name__ == '__main__':
	main()
