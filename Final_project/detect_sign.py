import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from KNN import image_preprocess

class signDetect(Node):
    def __init__(self):
        super().__init__('signDetect')

        # Load the KNN model
        self.model = cv2.ml.KNearest_load("...") # path to the model

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.detect_label,
            10)

        # Create the publisher for the sign state
        self.publisher = self.create_publisher(Int32, 'sign_state', 10)

        self.knn = cv2.ml.KNearest_create().load("")
        

    def detect_label(self, CompressedImage):
        imgBGR = CvBridge().imgmsg_to_cv2(CompressedImage, desired_enconding='bgr8')
        img = image_preprocess(imgBGR)
        sign = self.knn.findNearest(imgBGR, k=6)
        self.publisher.publish(sign)
        self.get_logger().info(f'Identified Sign: {sign}')
        

def main(args=None):
    rclpy.init(args=args)
    sign_detecter = signDetect()
    rclpy.spin(sign_detecter)
    sign_detecter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()