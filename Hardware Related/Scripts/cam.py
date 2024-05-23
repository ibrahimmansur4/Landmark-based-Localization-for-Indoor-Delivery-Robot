#!/usr/bin/env python3

import cv2
import numpy as np
import pytesseract

# Set the path to your Tesseract executable
pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'  # Replace with your Tesseract path

# Use the default camera (0 for the default camera, you can change it if needed)
cap = cv2.VideoCapture(0)

# Increase the window size
cv2.namedWindow("Laptop Camera Stream", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Laptop Camera Stream", 800, 600)  # Adjust the size as needed

# Define the center region of interest (ROI) coordinates
roi_x = 150
roi_y = 150
roi_width = 300
roi_height = 200

while True:
    ret, frame = cap.read()

    # Extract text using Tesseract from the center ROI
    roi = frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]
    text = pytesseract.image_to_string(roi, config='--psm 6')

    # Display the live stream with extracted text
    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.rectangle(frame, (roi_x, roi_y), (roi_x + roi_width, roi_y + roi_height), (255, 0, 0), 2)  # Draw ROI rectangle
    cv2.imshow("Laptop Camera Stream", frame)

    # Print the text serially
    print("Extracted Text:", text)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
