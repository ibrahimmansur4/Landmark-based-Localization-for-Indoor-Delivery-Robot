#!/usr/bin/env python3

import subprocess
import rospy
from geometry_msgs.msg import Twist

def start_turtlesim():
    subprocess.Popen(["roscore"])
    rospy.sleep(2)  # Wait for roscore to start
    subprocess.Popen(["rosrun", "turtlesim", "turtlesim_node"])

def move_turtle():
    # Initialize the ROS node
    rospy.init_node('circle_turtle_node', anonymous=True)

    # Create a publisher to send velocity commands to the turtle
    cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # Create a Twist message to set linear and angular velocities
    twist = Twist()
    twist.linear.x = 1.0  # Linear velocity in the x-axis
    twist.angular.z = 1.0  # Angular velocity around the z-axis

    # Publish the Twist message to move the turtle in a circle
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        cmd_vel_pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        start_turtlesim()
        move_turtle()
    except rospy.ROSInterruptException:
        pass
