#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pytesseract
from std_msgs.msg import String

class ImageProcessingNode:
    def __init__(self):
        rospy.init_node('image_processing_node', anonymous=True)

        # Subscribe to the camera image topic
        self.image_subscriber = rospy.Subscriber('/p3dx/front_camera/image_raw', Image, self.image_callback)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Publisher for the processed image
        self.image_publisher = rospy.Publisher('/image_processing/processed_image', Image, queue_size=10)

        # Publisher for the extracted text
        self.text_publisher = rospy.Publisher('/image_processing/extracted_text', String, queue_size=10)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform OCR on the image
            text = self.perform_ocr(cv_image)
            print("Extracted text: " , text)

            # Publish the extracted text
            self.text_publisher.publish(text)

            # Overlay text on the image
            cv2.putText(cv_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Convert the processed image back to ROS Image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

            # Publish the processed image
            self.image_publisher.publish(processed_image_msg)

        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

    def perform_ocr(self, image):
        # Extract text using Tesseract
        text = pytesseract.image_to_string(image, config='--psm 6')
        return text

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_processing_node = ImageProcessingNode()
        image_processing_node.run()
    except rospy.ROSInterruptException:
        pass