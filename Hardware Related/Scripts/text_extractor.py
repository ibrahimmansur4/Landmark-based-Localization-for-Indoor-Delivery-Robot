#!/usr/bin/env python3

import cv2
import numpy as np
import pytesseract
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Set the path to your Tesseract executable
pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'  # Replace with your Tesseract path

class TextExtractorNode:
    def __init__(self):
        rospy.init_node('text_extractor')
        self.publisher = rospy.Publisher('extracted_text', String, queue_size=10)
        self.bridge = CvBridge()
        self.roi_x = 150
        self.roi_y = 150
        self.roi_width = 300
        self.roi_height = 200

        # Initialize webcam
        self.cap = cv2.VideoCapture(0)  # 0 for default webcam, you can change it if you have multiple cameras

    def start(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("Failed to capture frame from webcam")
                continue

            # Extract the ROI
            roi = frame[self.roi_y:self.roi_y + self.roi_height, self.roi_x:self.roi_x + self.roi_width]

            # Convert the ROI to grayscale
            gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

            # Apply binary thresholding to enhance contrast
            _, thresh_roi = cv2.threshold(gray_roi, 150, 255, cv2.THRESH_BINARY)

            # Extract text using Tesseract with adjusted configuration
            text = pytesseract.image_to_string(thresh_roi, config='--psm 6')

            # Draw ROI rectangle
            cv2.rectangle(frame, (self.roi_x, self.roi_y), (self.roi_x + self.roi_width, self.roi_y + self.roi_height), (255, 0, 0), 2)

            # Add extracted text to the frame
            cv2.putText(frame, text, (self.roi_x, self.roi_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Publish the extracted text
            self.publisher.publish(text)

            # Display the webcam stream with bounding box, ROI, and extracted text
            cv2.imshow("Laptop Camera Stream", frame)
            cv2.waitKey(1)

            rate.sleep()

    def shutdown(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main():
    text_extractor = TextExtractorNode()
    try:
        text_extractor.start()
    except rospy.ROSInterruptException:
        text_extractor.shutdown()

if __name__ == '__main__':
    main()