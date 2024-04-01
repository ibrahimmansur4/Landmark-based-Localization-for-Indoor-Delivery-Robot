#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String

class ColorDetector:
    def __init__(self):
        rospy.init_node('color_detector_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.color_pub = rospy.Publisher('/color_detected', String, queue_size=10)

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            print(e)
            return

        # Define ROI coordinates (x, y, width, height)
        roi_x = 370
        roi_y = 380
        roi_width = 50
        roi_height = 60

        # Define the ROI
        roi = cv_image[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width]

        # Convert BGR image to HSV for the ROI
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Define range of colors to detect within the ROI
        # Red
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        # Light Green
        lower_light_green = np.array([60, 60, 60])
        upper_light_green = np.array([80, 255, 255])

        # Dark Green
        lower_dark_green = np.array([30, 40, 40])
        upper_dark_green = np.array([50, 255, 255])

        # Blue
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([140, 255, 255])

        # Yellow
        lower_yellow = np.array([10, 100, 100])
        upper_yellow = np.array([20, 255, 255])

        # Purple
        lower_purple = np.array([130, 40, 40])
        upper_purple = np.array([160, 255, 255])

        # Black
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([10, 10, 10])

        # Threshold the HSV image to get only desired colors within the ROI
        mask_red = cv2.inRange(hsv_roi, lower_red, upper_red)
        mask_light_green = cv2.inRange(hsv_roi, lower_light_green, upper_light_green)
        mask_dark_green = cv2.inRange(hsv_roi, lower_dark_green, upper_dark_green)
        mask_blue = cv2.inRange(hsv_roi, lower_blue, upper_blue)
        mask_yellow = cv2.inRange(hsv_roi, lower_yellow, upper_yellow)
        mask_purple = cv2.inRange(hsv_roi, lower_purple, upper_purple)
        mask_black = cv2.inRange(hsv_roi, lower_black, upper_black)

        # Bitwise-AND mask and original image for the ROI
        res_red = cv2.bitwise_and(roi, roi, mask=mask_red)
        res_light_green = cv2.bitwise_and(roi, roi, mask=mask_light_green)
        res_dark_green = cv2.bitwise_and(roi, roi, mask=mask_dark_green)
        res_blue = cv2.bitwise_and(roi, roi, mask=mask_blue)
        res_yellow = cv2.bitwise_and(roi, roi, mask=mask_yellow)
        res_purple = cv2.bitwise_and(roi, roi, mask=mask_purple)
        res_black = cv2.bitwise_and(roi, roi, mask=mask_black)

        # Find contours in the masks for the ROI
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_light_green, _ = cv2.findContours(mask_light_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_dark_green, _ = cv2.findContours(mask_dark_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_purple, _ = cv2.findContours(mask_purple, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_black, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contours on the original image for each color within the ROI
        cv2.rectangle(cv_image, (roi_x, roi_y), (roi_x+roi_width, roi_y+roi_height), (255, 0, 0), 2)  # Draw ROI rectangle
        cv2.drawContours(cv_image, contours_red, -1, (0, 0, 255), 2)  # Red
        cv2.drawContours(cv_image, contours_light_green, -1, (0, 255, 0), 2)  # Light Green
        cv2.drawContours(cv_image, contours_dark_green, -1, (0, 165, 0), 2)  # Dark Green
        cv2.drawContours(cv_image, contours_blue, -1, (255, 0, 0), 2)  # Blue
        cv2.drawContours(cv_image, contours_yellow, -1, (0, 255, 255), 2)  # Yellow
        cv2.drawContours(cv_image, contours_purple, -1, (128, 0, 128), 2)  # Purple
        cv2.drawContours(cv_image, contours_black, -1, (0, 0, 0), 2)  # Black

        # Display the detected colors in a single window
        cv2.imshow('Color Detection', cv_image)
        cv2.waitKey(1)

        # Output detected color
        detected_color = ""
        if len(contours_red) > 0:
            detected_color = "Red"
        elif len(contours_light_green) > 0:
            detected_color = "Light Green"
        elif len(contours_dark_green) > 0:
            detected_color = "Dark Green"
        elif len(contours_blue) > 0:
            detected_color = "Blue"
        # elif len(contours_yellow) > 0:
        #     detected_color = "Yellow"
        elif len(contours_purple) > 0:
            detected_color = "Purple"
        elif len(contours_black) > 0:
            detected_color = "Black"

        rospy.loginfo("Detected color: {}".format(detected_color))
        self.color_pub.publish(detected_color)

def main():
    color_detector = ColorDetector()
    rospy.spin()

if __name__ == '__main__':
    main()
