#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
import time
from tf.transformations import euler_from_quaternion

class RobotMover:
    def __init__(self, linear_velocity=0.5, angular_velocity=0.2):
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.cmd_vel_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
        rospy.init_node('move_robot_node', anonymous=True)
        self.odom_subscriber = rospy.Subscriber('/RosAria/odom', Odometry, self.odom_callback)
        self.current_odom = Odometry()

        # Subscribe to sonar topic
        self.sonar_subscriber = rospy.Subscriber('/sonar_scan', Range, self.sonar_callback)
        # Initialize sonar_data attribute
        self.sonar_data = Range()

        # Variables for dead reckoning
        self.prev_time = time.time()
        self.prev_x = 0.0
        self.prev_y = 0.0
        
        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.5
        self.safe_distance = 0.8
        self.wall_follow_distance = 0.7
        self.rooms = {
            "Room1": (-1.834246, 1.440730),
            "Room2": (-1.861298, 3.310347),
            "Room3": (-1.861288, 5.353190),
            "Room4": (2.624382, 5.353169),
            "Room5": (2.644843, 3.443352),
            "Room6": (2.644833, 1.440473),
            "DockingStation": (-4.571235, -6.670997)
        }
        self.room_colors = {
            "Room1": "Red",
            "Room2": "Blue",
            "Room3": "Light Green",
            "Room4": "Purple",
            "Room5": "Black",
            "Room6": "Yellow",
            "DockingStation": "Dark Green"
        }
        rospy.spin()
        # Wait for initial Odometry values to be received
        rospy.sleep(1.0)

    def publish_twist_command(self, twist_cmd):
        self.cmd_vel_pub.publish(twist_cmd)
        rospy.sleep(0.1)  # Publish at 10 Hz

    def odom_callback(self, msg):
        self.current_odom = msg

        # Update dead reckoning variables
        current_time = time.time()
        delta_time = current_time - self.prev_time

        linear_velocity = self.current_odom.twist.twist.linear.x
        angular_velocity = self.current_odom.twist.twist.angular.z

        delta_x = linear_velocity * delta_time * math.cos(angular_velocity)
        delta_y = linear_velocity * delta_time * math.sin(angular_velocity)

        self.prev_x += delta_x
        self.prev_y += delta_y

        self.prev_time = current_time

    def sonar_callback(self, sonar_data):
        # Callback function to process sonar data and avoid obstacles

        # Get the distance from the sonar sensor
        distance = sonar_data.range

        # Check if there is an obstacle within the safe distance
        if distance < self.safe_distance:
            # Obstacle detected
            self.move_around_obstacle()
        else:
            # No obstacle
            self.wall_follow(sonar_data)
            
    def wall_follow(self, sonar_data):
        # Wall-following behavior to maintain a certain distance from the wall

        # Get the distance from the side sonar sensor (assumes sonar_data is published as a Range message)
        side_distance = sonar_data.range

        # Calculate the error (difference between the desired and actual distance)
        error = side_distance - self.wall_follow_distance

        # Proportional control: Adjust angular velocity based on the error
        angular_velocity = -0.5 * error  # Adjust the proportional control gain as needed

        # Cap the angular velocity to stay within a reasonable range
        angular_velocity = max(min(angular_velocity, self.max_angular_speed), -self.max_angular_speed)

        # Move forward with a constant linear velocity
        twist_msg = Twist()
        twist_msg.linear.x = self.max_linear_speed
        twist_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist_msg)

    def move_around_obstacle(self):
        # Move around the obstacle by turning
        twist_msg = Twist()
        twist_msg.linear.x = 0.0  # Stop linear motion
        twist_msg.angular.z = self.max_angular_speed  # Rotate to move around the obstacle
        self.cmd_vel_pub.publish(twist_msg)

    def move_linear_and_rotate(self, linear_distance, target_angle):
        rospy.loginfo(f"Moving linear distance: {linear_distance} meters and rotating to angle: {target_angle} radians")

        twist_cmd = Twist()
        twist_cmd.linear.x = self.linear_velocity
        twist_cmd.angular.z = self.angular_velocity

        start_x = self.current_odom.pose.pose.position.x
        start_y = self.current_odom.pose.pose.position.y
        start_orientation = euler_from_quaternion([
            self.current_odom.pose.pose.orientation.x,
            self.current_odom.pose.pose.orientation.y,
            self.current_odom.pose.pose.orientation.z,
            self.current_odom.pose.pose.orientation.w
        ])[2]

        while not rospy.is_shutdown():
            current_x = self.current_odom.pose.pose.position.x
            current_y = self.current_odom.pose.pose.position.y
            current_distance = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)

            current_orientation = euler_from_quaternion([
                self.current_odom.pose.pose.orientation.x,
                self.current_odom.pose.pose.orientation.y,
                self.current_odom.pose.pose.orientation.z,
                self.current_odom.pose.pose.orientation.w
            ])[2]

            # Move linearly
            if current_distance < linear_distance:
                self.publish_twist_command(twist_cmd)
            else:
                twist_cmd.linear.x = 0.0
                self.publish_twist_command(twist_cmd)

            # Rotate to the target angle
            angle_difference = target_angle - (current_orientation - start_orientation)
            twist_cmd.angular.z = math.copysign(self.angular_velocity, angle_difference)

            # Ensure the robot stops rotating when it reaches the target angle
            if abs(angle_difference) < 0.01:
                twist_cmd.angular.z = 0.0

            # Break the loop when both linear and rotational motions are completed
            if current_distance >= linear_distance and abs(angle_difference) < 0.01:
                break

        # Stop the robot by publishing a Twist message with zero velocities
        twist_cmd.linear.x = 0.0
        twist_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_cmd)

    def move_autonomously(self):
        # Move autonomously and explore while avoiding obstacles
        rospy.loginfo("Starting autonomous exploration. Press Ctrl+C to stop.")
        while not rospy.is_shutdown():
            # Move the robot forward aimlessly
            distance = 2.0  # Set your desired distance
            angle = 0.0  # Move straight forward
            self.move_linear_and_rotate(distance, angle)

            # After moving forward, adjust the position based on obstacle avoidance
            self.sonar_callback(self.sonar_data)

if __name__ == '__main__':
    try:
        robot_mover = RobotMover(linear_velocity=0.5, angular_velocity=0.2)
        robot_mover.move_autonomously()
    except rospy.ROSInterruptException:
        pass
