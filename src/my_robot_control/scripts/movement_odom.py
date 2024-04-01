#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

class RobotMover:
    def __init__(self, linear_velocity=1.0, angular_velocity=0.0):
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        # self.cmd_vel_pub = rospy.Publisher('/p3dx/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
        rospy.init_node('move_robot_node', anonymous=True)

    def publish_twist_command(self, twist_cmd):
        self.cmd_vel_pub.publish(twist_cmd)
        rospy.sleep(0.1)  # Publish at 10 Hz

    def move_for_duration(self, duration):
        twist_cmd = Twist()
        twist_cmd.linear.x = self.linear_velocity
        twist_cmd.angular.z = self.angular_velocity

        start_time = time.time()
        while not rospy.is_shutdown() and (time.time() - start_time) < duration:
            self.publish_twist_command(twist_cmd)

        # Stop the robot by publishing a Twist message with zero velocities
        twist_cmd.linear.x = 0.0
        twist_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_cmd)

def main():
    try:
        robot_mover = RobotMover(linear_velocity=1.0, angular_velocity=0.0)
        robot_mover.move_for_duration(duration=5.0)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
