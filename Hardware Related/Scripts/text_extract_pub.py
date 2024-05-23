#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

class LocationSubscriberNode:
    def __init__(self):
        rospy.init_node('location_subscriber')
        self.subscriber = rospy.Subscriber('extracted_text', String, self.callback)
        self.publisher = rospy.Publisher('current_room', Point, queue_size=10)
        self.room_coordinates = {
            "A-101": (-2.59, -4.31),
            "C-110": (2.59, -7.71),   
            "C-107": (-2.59, 5.60),   
        }

    def extract_valid_room_numbers(self, text):
        valid_room_numbers = []
        for word in text.split():
            if word.upper() in self.room_coordinates:
                valid_room_numbers.append(word.upper())
        return valid_room_numbers

    def callback(self, data):
        extracted_text = data.data
        valid_room_numbers = self.extract_valid_room_numbers(extracted_text)
        
        if valid_room_numbers:
            for room_number in valid_room_numbers:
                coordinates = self.room_coordinates[room_number]
                rospy.loginfo(f"Room Number: {room_number}, Coordinates: {coordinates}")
                self.publish_coordinates(coordinates)
        else:
            rospy.logwarn("No valid room numbers found in the extracted text.")

    def publish_coordinates(self, coordinates):
        point = Point()
        point.x, point.y = coordinates
        self.publisher.publish(point)

    def spin(self):
        rospy.spin()

def main():
    location_subscriber = LocationSubscriberNode()
    location_subscriber.spin()

if __name__ == '__main__':
    main()

