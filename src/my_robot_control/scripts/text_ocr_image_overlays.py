#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pytesseract

class ImageProcessingNode:
    def __init__(self):
        rospy.init_node('image_processing_node', anonymous=True)

        # Subscribe to the camera image topic
        self.image_subscriber = rospy.Subscriber('/p3dx/front_camera/image_raw', Image, self.image_callback)

        # Advertise a new topic to publish the image with text overlay
        self.overlay_pub = rospy.Publisher('/image_with_overlay', Image, queue_size=10)

        # Initialize CvBridge
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform OCR on the image
            text = self.perform_ocr(cv_image)
            print("Extracted text: " ,text)
            # Overlay the text on the image
            image_with_overlay = self.overlay_text(cv_image, text)

            # Publish the image with overlay
            self.publish_overlay(image_with_overlay)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", str(e))
        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

    def perform_ocr(self, image):
        # Extract text using Tesseract
        text = pytesseract.image_to_string(image, config='--psm 6')
        return text

    def overlay_text(self, image, text):
        # Overlay the text on the image
        cv2.putText(image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        return image

    def publish_overlay(self, image):
        try:
            # Convert OpenCV image to ROS Image message
            overlay_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')

            # Publish the image with overlay
            self.overlay_pub.publish(overlay_msg)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", str(e))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_processing_node = ImageProcessingNode()
        image_processing_node.run()
    except rospy.ROSInterruptException:
        pass
