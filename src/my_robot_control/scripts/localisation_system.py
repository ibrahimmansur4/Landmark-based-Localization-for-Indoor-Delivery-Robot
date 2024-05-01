#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import String
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class RobotMover:
    def __init__(self, linear_velocity, angular_velocity):
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.cmd_vel_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
        rospy.init_node('move_robot_node', anonymous=True)
        self.odom_subscriber = rospy.Subscriber('/RosAria/odom', Odometry, self.odom_callback)
        self.sonar_subscriber = rospy.Subscriber('/sonar_scan', Range, self.sonar_callback)
        self.color_sub = rospy.Subscriber('/color_detected', String, self.color_callback)

        self.current_odom = None
        self.sonar_data = None
        self.color_detected = None
        self.previous_pose = None
        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.5
        self.wall_follow_distance = 0.8


            
        self.rooms = {
        "Room1": {"position": (-1.834246, 1.440730), "orientation": (0, 0, 0.707, -0.707)},
        "Room2": {"position": (-1.861298, 3.310347), "orientation": (0, 0, 0.707, -0.707)},
        "Room3": {"position": (-1.861288, 5.353190), "orientation": (0, 0, 0.707, -0.707)},
        "Room4": {"position": (2.624382, 5.353169), "orientation": (0, 0, 0.707, 0.707)},
        "Room5": {"position": (2.644843, 3.443352), "orientation": (0, 0, 0.707, 0.707)},
        "Room6": {"position": (2.644833, 1.440473), "orientation": (0, 0, 0.707, 0.707)},
        "DockingStation": {"position": (-4.571235, -6.670997), "orientation": (0, 0, 0.707, 0.707)}
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

        self.goal_position = self.rooms["Room1"]
        self.state = "wall_follow"
        self.angular_tolerance = 0.1
    def publish_twist_command(self, twist_cmd):
        self.cmd_vel_pub.publish(twist_cmd)
        rospy.sleep(0.1)  # Publish at 10 Hz

    def odom_callback(self, msg):
        self.current_odom = msg

    def sonar_callback(self, sonar_data):
        self.sonar_data = sonar_data

    def color_callback(self, msg):
        self.color_detected = msg.data
        

    def wall_follow(self):
        if self.sonar_data is None:
            return

        side_distance = self.sonar_data.range
        error = side_distance - self.wall_follow_distance
        angular_velocity = -0.5 * error
        angular_velocity = max(min(angular_velocity, self.max_angular_speed), -self.max_angular_speed)

        twist_msg = Twist()
        twist_msg.linear.x = 0.8  # Reduced speed for detecting landmark
        twist_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist_msg)

    def detecting_landmark(self):
        if self.color_detected is not None:
            color = self.color_detected
            # Check if the detected landmark is the same as the goal
            self.goal_room = list(self.rooms.keys())[list(self.rooms.values()).index(self.goal_position)]
            if color in self.room_colors.values():
                room_name = list(self.room_colors.keys())[list(self.room_colors.values()).index(color)]
                if room_name == self.goal_room:
                    self.state = "stop"
                    rospy.loginfo("Detected goal landmark, stopping.")
                else:
                    self.start_position = self.rooms[room_name]
                    self.state = "go_to_goal"
                    rospy.loginfo(f"Detected {color} color, starting position set to {self.start_position}, state set to {self.state}")
            else:
                rospy.logwarn(f"Unknown color detected: {color}")
                self.state = "wall_follow"
                rospy.loginfo(f"State set to {self.state}")
        else:
            self.state = "wall_follow"
            rospy.loginfo(f"No color detected, state set to {self.state}")

    def go_to_goal(self):
        start_x, start_y = self.start_position["position"]
        goal_x, goal_y = self.goal_position["position"]

        # Use start position as current position
        current_x, current_y = start_x, start_y

        twist_msg = Twist()

        # Extract the quaternion components and pass them to euler_from_quaternion
        current_quaternion = [
            self.current_odom.pose.pose.orientation.x,
            self.current_odom.pose.pose.orientation.y,
            self.current_odom.pose.pose.orientation.z,
            self.current_odom.pose.pose.orientation.w
        ]
        _, _, current_yaw = euler_from_quaternion(current_quaternion)


        # Calculate angle to rotate towards the goal
        goal_angle = math.atan2(goal_y - current_y, goal_x - current_x)
        angle_difference = goal_angle - current_yaw

        # Ensure angle is between -pi and pi for proper rotation
        while angle_difference > math.pi:
            angle_difference -= 2 * math.pi
        while angle_difference < -math.pi:
            angle_difference += 2 * math.pi

        # Rotate the robot to face the goal
        if angle_difference > 0:
            twist_msg.angular.z = self.angular_velocity  # Rotate counter-clockwise
        else:
            twist_msg.angular.z = -self.angular_velocity  # Rotate clockwise

        while abs(angle_difference) > self.angular_tolerance:
            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(0.1)  # Sleep for a short time to allow the robot to rotate
            # Update current_quaternion and current_yaw
            current_quaternion = [
                self.current_odom.pose.pose.orientation.x,
                self.current_odom.pose.pose.orientation.y,
                self.current_odom.pose.pose.orientation.z,
                self.current_odom.pose.pose.orientation.w
            ]
            _, _, current_yaw = euler_from_quaternion(current_quaternion)

            angle_difference = goal_angle - current_yaw

            # Ensure angle is between -pi and pi for proper rotation
            while angle_difference > math.pi:
                angle_difference -= 2 * math.pi
            while angle_difference < -math.pi:
                angle_difference += 2 * math.pi
                
            rospy.loginfo(f"Angle difference: {angle_difference}")  # Log the angle difference for debugging
        # Stop rotating once facing the goal
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

        # Calculate distance to goal and time to reach goal based on robot's speed
        current_distance = math.sqrt((current_x - goal_x)**2 + (current_y - goal_y)**2)
        time_to_goal = current_distance / self.linear_velocity
        rospy.loginfo(f"Time: : {time_to_goal}")
        
        twist_msg.linear.x = self.linear_velocity

        # Start a timer
        start_time = rospy.Time.now().to_sec()
        time_moved = 0.0
        remaining_time = time_to_goal

        while remaining_time > 0.0 and not rospy.is_shutdown():
            if self.sonar_data is not None and self.sonar_data.range < 0.9:  # Obstacle detected
                # Save the time moved and calculate remaining time
                time_moved += rospy.Time.now().to_sec() - start_time
                remaining_time = time_to_goal - time_moved # Log the remaining time for debugging
                rospy.loginfo(f"Remaining time: {remaining_time}")
                if time_moved < 0.7 * time_to_goal:  # Only handle obstacle if less than 3/4 of the total time has elapsed
                    twist_msg.linear.x = 0.0  # Stop the robot
                    self.cmd_vel_pub.publish(twist_msg)
                    rospy.logwarn("Obstacle detected, stopping robot.")

                    # Wait for the obstacle to be removed
                    while self.sonar_data.range < 0.5 and not rospy.is_shutdown():
                        rospy.sleep(0.1)

                    rospy.loginfo("Obstacle removed, continuing to goal.")
                    start_time = rospy.Time.now().to_sec()  # Reset start time
                    twist_msg.linear.x = self.linear_velocity  # Start moving again
                else:
                    # Stop the robot completely
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.0  # Set angular velocity to 0
                    self.cmd_vel_pub.publish(twist_msg)

                    # Wait until the robot has stopped
                    while not self.is_robot_stopped():
                        rospy.sleep(0.1)
                        self.cmd_vel_pub.publish(twist_msg)
                        
                    self.cmd_vel_pub.publish(twist_msg)
                    self.state = "wall_follow"
                    rospy.logwarn("Wall detected, starting wall follow.")
                    
                    break
                    # Check if the goal landmark is the same color as expected
                    # goal_room = list(self.rooms.keys())[list(self.rooms.values()).index(self.goal_position)]
                    # if self.color_detected == self.room_colors[goal_room]:
                    #     self.state = "stop"
                    #     rospy.loginfo("Reached goal position, stopping.")
                    #     twist_msg.linear.x = 0.0
                    #     self.cmd_vel_pub.publish(twist_msg)
                    #     break
                    
                        

            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(0.1)  # Sleep for a short time to allow the robot to move

        # Stop the robot if it reached the goal or if an obstacle was detected and handled
        if self.state != "stop" and self.state != "wall_follow":
            self.state = "stop"
            rospy.loginfo("Reached goal position, stopping.")
            twist_msg.linear.x = 0.0
            self.cmd_vel_pub.publish(twist_msg)
            
    def stop(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)

    def is_robot_stopped(self):
    # Check if the robot has stopped moving by comparing its current and previous odometry positions.
        current_pose = self.current_odom.pose.pose.position
        if self.previous_pose is None:
            self.previous_pose = current_pose
            return False

        position_diff = math.sqrt(
            (current_pose.x - self.previous_pose.x) ** 2 +
            (current_pose.y - self.previous_pose.y) ** 2
        )

        if position_diff < 0.01:  # Adjust this threshold as needed
            self.previous_pose = current_pose
            return True
        else:
            self.previous_pose = current_pose
            return False
    
    def move_autonomously(self):
        rospy.loginfo("Starting autonomous exploration. Press Ctrl+C to stop.")
        rate = rospy.Rate(1)  # 1 Hz rate for state transitions

        while not rospy.is_shutdown():
            if self.state == "wall_follow":
                for _ in range(3):  # Wall follow for 3 seconds
                    rospy.loginfo("Wall follow state.")
                    self.wall_follow()
                    rospy.sleep(1.0)
                self.state = "detecting_landmark"
                rospy.loginfo("Changed state to detect.")
            elif self.state == "detecting_landmark":
                rospy.loginfo("Detect Landmark state.")
                self.detecting_landmark()
                rospy.sleep(1.0)  # Read landmark for 1 second
            elif self.state == "go_to_goal":
                rospy.loginfo("Go to goal state.")
                self.go_to_goal()
            elif self.state == "stop":
                rospy.loginfo("Stop state.")
                self.stop()
            rate.sleep()

if __name__ == '__main__':
    try:
        robot_mover = RobotMover(0.5,0.5)
        robot_mover.move_autonomously()
    except rospy.ROSInterruptException:
        pass