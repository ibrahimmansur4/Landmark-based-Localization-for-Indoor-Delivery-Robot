#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pytesseract

class ImageProcessingNode:
    def __init__(self):
        rospy.init_node('image_processing_node', anonymous=True)

        # Subscribe to the camera image topic
        self.image_subscriber = rospy.Subscriber('/p3dx/front_camera/image_raw', Image, self.image_callback)

        # Initialize CvBridge
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform OCR on the image
            text, bounding_boxes = self.perform_ocr(cv_image)

            # Draw bounding boxes around text
            for bbox in bounding_boxes:
                x, y, w, h = bbox
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Display the image with bounding boxes
            cv2.imshow("Image with Bounding Boxes", cv_image)
            cv2.waitKey(1)

            # Print the extracted text
            rospy.loginfo("Extracted Text: %s", text)

        except Exception as e:
            rospy.logerr("Error processing image: %s", str(e))

    def perform_ocr(self, image):
        # Extract text using Tesseract
        text = pytesseract.image_to_string(image, config='--psm 6')

        # Find bounding boxes around text regions
        h, w, _ = image.shape
        bounding_boxes = pytesseract.image_to_boxes(image)

        # Parse bounding box coordinates
        bounding_boxes = [(int(box.split()[1]), h - int(box.split()[2]), int(box.split()[3]), h - int(box.split()[4])) for box in bounding_boxes]

        return text, bounding_boxes

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_processing_node = ImageProcessingNode()
        image_processing_node.run()
    except rospy.ROSInterruptException:
        pass
