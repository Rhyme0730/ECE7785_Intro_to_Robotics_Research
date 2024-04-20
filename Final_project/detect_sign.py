import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from KNN import image_preprocess
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Point

class signDetect(Node):
    def __init__(self):
        super().__init__('signDetect')
        image_qos_profile = QoSProfile(
		    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.detect_label,
            image_qos_profile)

        # Create the publisher for the sign state
        self.publisher = self.create_publisher(Point, 'sign_state', 10)
        self.knn = cv2.ml.KNearest_load("/home/fayne/ECE7785-main/Final_project/knn")
        print('successfully load knn model')

    def detect_label(self, CompressedImage):
        imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, 'bgr8')
        img = image_preprocess(imgBGR, 30, 30)
        # sign = self.knn.findNearest(img, k=6)
        features = np.array(img)
        features = img.flatten().reshape(1, 30*30*1)
        features = features.astype(np.float32)

        sign, results, neighbours, dist = self.knn.findNearest(features, cv2.ml.COL_SAMPLE, 6)
        msg = Point()
        msg.x = sign
        self.publisher.publish(msg)
        # self.get_logger().info(f'Identified Sign: {msg.x}')
        print(f'detect sign{msg.x}')
        

def main(args=None):
    rclpy.init(args=args)
    sign_detecter = signDetect()
    rclpy.spin(sign_detecter)
    sign_detecter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# [-3.22, 0.15],  # 0
#                                    [-2.96, -0.79], # 1
#                                    [-4.02, -0.95], # 2
#                                    [-4.21, 0.00], # 3
#                                    [-4.12, 1.18], # 4
#                                    [-3.13, 1.17], # 5
#                                    [-2.15, 1.04], # 6:goal
#                                    [-2.12, 0.17], # 7
#                                    [-2.34, -0.79], # 8 
#                                    [-1.31, -0.94], # 9: right
#                                    [-1.43, 0.13], # 10: backward
#                                    [-1.34, 1.00], # 11: backward
#                                    [-0.38, 1.14], # 12: left
#                                    [-0.44, -0.04], # 13: right
#                                    [0.49, 0.19], # 14: left
#                                    [], # 15: forward
#                                    [] # 16: left
