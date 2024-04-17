import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class SignIdentifier(Node):
    """
    Create a SignIdentifier class, which is a subclass of the Node class.
    """
    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('identify_sign')

        # Load the KNN model
        self.model = cv2.ml.KNearest_load("/home/syqua/KNN/knn_maze") # path to the model

        # Create the subscriber. This subscriber will receive an Image
        # from the image_raw topic. The callback function is called
        # when a message is received.
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)

        # Create the publisher for the sign state
        self.publisher = self.create_publisher(Int32, 'sign_state', 10)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def crop_image(self,image):
        # Convert image to HSV color space
        image = image[:][math.floor(240*0.25):]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (7, 7),0)


        # Define color ranges
        lower = np.array([0, 120, 20])
        upper = np.array([180, 255, 200])

        # Threshold the HSV image to get binary mask
        mask = cv2.inRange(hsv, lower, upper)
        combined_mask = mask
        
        # Apply dilation and erosion to the combined mask for noise reduction
        kernel = np.ones((5, 5), np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
    #     combined_mask = combined_mask[:][:math.floor(240*0.8)

        # Find contours in the combined mask
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #     contours, _ = cv2.findContours(combined_mask[:][:math.floor(240*0.8)], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        # Filter contours based on area threshold
        min_area_threshold = 100  # Adjust as needed
        filtered_contours = [contour for contour in contours if cv2.contourArea(contour) > min_area_threshold]

        # Find the contour with the largest area (presumably the blob)
        if filtered_contours:
            contour = max(filtered_contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(contour)
            # Crop the original image based on the bounding box of the contour
            cropped_image = image[y:y+h, x:x+w]
            cropped_mask = combined_mask[y:y+h, x:x+w]
            gray_cropped_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
        else:
    #         # If no contour is found, return the original image
            cropped_image = image
            cropped_mask = combined_mask
            gray_cropped_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
            # If no contour is found, create a black image with the same dimensions as the original image
    #         cropped_image = np.zeros_like(image)

        return cropped_image,gray_cropped_image,cropped_mask

    def listener_callback(self, msg):
        """
        Callback function to process the received image.
        """
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Preprocess the image
        cropped_image = self.crop_image(current_frame)[1]
        resized_image = cv2.resize(cropped_image, (40, 40))
        features = resized_image.flatten().reshape(1, 40*40*1)

        # Convert test data to CV_32F data type
        features = np.array(features, dtype=np.float32)

        # Predict the label using the KNN model
        ret, _, _, _ = self.model.findNearest(features.reshape(1, -1), 5)
        label = int(ret)

        # Publish the identified sign label
        self.publisher.publish(Int32(data=label))

        # Log the predicted label
        self.get_logger().info(f'Identified Sign: {label}')

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    sign_identifier = SignIdentifier()

    # Spin the node so the callback function is called.
    rclpy.spin(sign_identifier)

    # Destroy the node explicitly
    sign_identifier.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    