#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String

class Bug2Algorithm:
    def __init__(self):
        rospy.init_node('bug2_node', anonymous=True)   
        # Initialize subscribers
        #In p3dx the sonar sensor works as a single subscriber with range/arrays of sensor like a laser scan
        #Will have to modify for the hardware implementation
        self.sonar_subscriber_1 = rospy.Subscriber('/sonar_scan_1', Range, self.sonar_callback_1)
        self.sonar_subscriber_2 = rospy.Subscriber('/sonar_scan_2', Range, self.sonar_callback_2)
        self.sonar_subscriber_3 = rospy.Subscriber('/sonar_scan_3', Range, self.sonar_callback_3)
        self.sonar_subscriber_5 = rospy.Subscriber('/sonar_scan_5', Range, self.sonar_callback_5)
        self.sonar_subscriber_6 = rospy.Subscriber('/sonar_scan_6', Range, self.sonar_callback_6)
        self.odom_subscriber = rospy.Subscriber('/RosAria/odom', Odometry, self.odom_callback)
        # Color detection
        self.color_sub = rospy.Subscriber('/color_detected', String, self.color_callback)
        
        # Initialize publishers
        self.cmd_vel_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)

        # Initialize variables
        self.current_odom = None
        self.sonar_data = {
            'fright': 0,
            'front_right': 0,
            'front': 0,
            'front_left': 0,
            'fleft': 0,
        }
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
        self.detected_color = None
        # Initialize variables
        self.start_pose = (0, 0)  # Set the start coordinates here
        self.target_pose = (-1.0, 3.3)  # Set the goal coordinates here
        self.intermediate_pose = (-1.0, 1.4)  # Set the intermediate coordinates here
        self.heading = -90 * math.pi / 180  # Set the desired heading at the intermediate pose (in radians)
        self.state = None  # Initialize state to None
        self.previous_state = None
        self.is_return_journey = False
        self.hit_point = None
        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.5
        self.safe_distance = 0.8
        
        # Start-Goal Line Calculated?
        self.start_goal_line_calculated = False
        
        # Start-Goal Line Parameters
        self.start_goal_line_slope_m = 0
        self.start_goal_line_y_intercept = 0
        self.start_goal_line_xstart = 0
        self.start_goal_line_xgoal = 0
        self.start_goal_line_ystart = 0
        self.start_goal_line_ygoal = 0
        # Leave point must be within +/- 0.1m of the start-goal line
        # in order to go from wall following mode to go to goal mode
        self.distance_to_start_goal_line_precision = 0.1
        # Used to record the (x,y) coordinate where the robot hit
        # a wall.
        self.hit_point_x = 0
        self.hit_point_y = 0
        # Distance between the hit point and the goal in meters
        self.distance_to_goal_from_hit_point = 0.0
        # Used to record the (x,y) coordinate where the robot left
        # a wall.       
        self.leave_point_x = 0
        self.leave_point_y = 0
        
        # Distance between the leave point and the goal in meters
        self.distance_to_goal_from_leave_point = 0.0
        
        # The hit point and leave point must be far enough 
        # apart to change state from wall following to go to goal
        # This value helps prevent the robot from getting stuck and
        # rotating in endless circles.
        # This distance was determined through trial and error.
        self.leave_point_to_hit_point_diff = 0.25 # in meters
        
        
    def sonar_callback_1(self, data):
        self.sonar_data['fright'] = data.range

    def sonar_callback_2(self, data):
        self.sonar_data['front_right'] = data.range

    def sonar_callback_3(self, data):
        self.sonar_data['front'] = data.range

    def sonar_callback_5(self, data):
        self.sonar_data['front_left'] = data.range

    def sonar_callback_6(self, data):
        self.sonar_data['fleft'] = data.range

    def odom_callback(self, msg):
        self.current_odom = msg

    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    def color_callback(self, msg):
        self.detected_color = msg.data
        
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.current_odom is None or not self.sonar_data:
                continue

            current_x = self.current_odom.pose.pose.position.x
            current_y = self.current_odom.pose.pose.position.y

            if self.state is None:
                rospy.loginfo("Setting initial state to 'go_to_intermediate'")
                self.state = 'go_to_intermediate'

            if self.state == 'go_to_intermediate':
                rospy.loginfo("In state 'go_to_intermediate'")
                # Go towards the intermediate pose
                self.move_towards_pose(self.intermediate_pose[0], self.intermediate_pose[1])

                # Check if the intermediate pose is reached
                distance_to_intermediate = self.euclidean_distance(current_x, current_y,
                                                                    self.intermediate_pose[0], self.intermediate_pose[1])
                if distance_to_intermediate < 0.1:
                    rospy.loginfo("Reached intermediate pose")
                    self.adjust_heading(self.heading)  # Adjust heading to the desired value

                    # Check if the expected color is detected
                    if self.detected_color == 'Red':
                        rospy.loginfo("Detected red color, confirming intermediate pose")
                        self.state = 'go_to_goal'
                    else:
                        rospy.logwarn("Expected red color not detected, adjusting position")
                        self.adjust_position_intermediate()

            elif self.state == 'go_to_goal':
                rospy.loginfo("In state 'go_to_goal'")
                # Go towards the goal
                self.move_towards_pose(self.target_pose[0], self.target_pose[1])

                # Check if the goal is reached
                distance_to_goal = self.euclidean_distance(current_x, current_y, self.target_pose[0], self.target_pose[1])
                if distance_to_goal < 0.1:
                    rospy.loginfo("Reached goal")
                    self.adjust_heading(self.heading)  # Adjust heading to 270 degrees (-90 degrees)

                    # Check if the expected color is detected
                    if self.detected_color == 'Blue':
                        rospy.loginfo("Detected blue color, confirming goal pose")
                        self.is_return_journey = True
                        self.state = 'go_to_start'
                    else:
                        rospy.logwarn("Expected blue color not detected, adjusting position")
                        self.adjust_position_goal()

            elif self.state == 'go_to_start':
                rospy.loginfo("In state 'go_to_start'")
                # Return to the start pose
                self.move_towards_pose(self.start_pose[0], self.start_pose[1])

                # Check if the start pose is reached
                distance_to_start = self.euclidean_distance(current_x, current_y, self.start_pose[0], self.start_pose[1])
                if distance_to_start < 0.1:
                    rospy.loginfo("Returned to start")
                    self.adjust_heading(self.heading)

                    # Check if the expected color is detected
                    if self.detected_color == 'Green':
                        rospy.loginfo("Detected green color, confirming start pose")
                        self.stop_robot()
                        break
                    else:
                        rospy.logwarn("Expected green color not detected, adjusting position")
                        self.adjust_position_start()

            elif self.state == 'follow_boundary':
                rospy.loginfo("In state 'follow_boundary'")
                # Follow the boundary of the obstacle
                self.follow_the_wall()

            # Check for obstacle detection and update state if necessary
            if self.sonar_data['front'] < self.safe_distance:
                if self.state != 'follow_boundary':
                    self.previous_state = self.state  # Store the current state as previous_state
                    rospy.loginfo("Obstacle detected, switching to 'follow_boundary' state")
                    # Obstacle detected, update state and hit point
                    self.state = 'follow_boundary'
                    self.hit_point = (current_x, current_y)
                    hit_point_x, hit_point_y = self.hit_point

            rate.sleep()

    def adjust_position_intermediate(self):
        # Move the robot slightly forward
        while self.detected_color != 'Red':
            twist_msg = Twist()
            twist_msg.linear.x = 1.0
            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(1)  # Adjust sleep time as needed for the desired movement duration
            twist_msg.linear.x = -1.0  # Stop linear movement
            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(2)
            
        self.detected_color = 'Red'
        
    def adjust_position_goal(self):
        # Move the robot slightly forward
        #Will have to add checks for the colors so only the goal color is the 
        #one that is detected
        #The while loop should have an if statement that checks if the color is
        #the one associated with the room
        while self.detected_color != 'Blue': #hardcoded for now
            twist_msg = Twist()
            twist_msg.linear.x = 1.0
            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(1)  # Adjust sleep time as needed for the desired movement duration
            twist_msg.linear.x = -1.0  # Stop linear movement
            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(2)
            
        self.detected_color = 'Blue'

    def adjust_position_start(self):
        # Move the robot slightly forward
        self.move_towards_pose(self.start_pose[0], self.start_pose[1], distance_offset=0.5)

        
    def move_towards_pose(self, goal_x, goal_y, distance_offset=0.0):
        current_x = self.current_odom.pose.pose.position.x
        current_y = self.current_odom.pose.pose.position.y

        # Calculate the direction to the goal
        dx = goal_x - current_x
        dy = goal_y - current_y
        goal_angle = math.atan2(dy, dx)

        # Calculate the difference between the current orientation and the desired orientation
        current_orientation = self.get_current_orientation()
        angle_difference = goal_angle - current_orientation

        # Adjust the goal position based on the distance_offset
        adjusted_goal_x = goal_x + distance_offset * math.cos(goal_angle)
        adjusted_goal_y = goal_y + distance_offset * math.sin(goal_angle)

        # Publish twist command to move towards the adjusted goal
        twist_cmd = Twist()
        twist_cmd.linear.x = self.max_linear_speed * math.cos(angle_difference)
        twist_cmd.angular.z = self.max_angular_speed * angle_difference
        self.cmd_vel_pub.publish(twist_cmd)

    def adjust_heading(self, target_heading):
        # Adjust the robot's heading to the target_heading once
        current_heading = self.get_current_orientation()
        heading_difference = target_heading - current_heading

        twist_cmd = Twist()
        twist_cmd.angular.z = self.max_angular_speed * heading_difference

        rate = rospy.Rate(10)  # 10 Hz
        timeout = rospy.Time.now() + rospy.Duration(15)  # Timeout after 15 seconds

        while not rospy.is_shutdown() and abs(heading_difference) > 0.05 and rospy.Time.now() < timeout:
            self.cmd_vel_pub.publish(twist_cmd)
            current_heading = self.get_current_orientation()
            heading_difference = target_heading - current_heading
            rospy.loginfo("Adjusting Heading: %.2f", heading_difference)
            # Check for color detection while adjusting heading
            # if self.detected_color is not None:
            #     rospy.loginfo("Color detected while adjusting heading: %s", self.detected_color)
            #     break

            rate.sleep()

        # Stop the robot after adjusting heading
        self.cmd_vel_pub.publish(Twist())

        # Reset some variables
        self.start_goal_line_calculated = False
        

    def get_current_orientation(self):
        # Get the current orientation of the robot from the odometry data
        quaternion = (
            self.current_odom.pose.pose.orientation.x,
            self.current_odom.pose.pose.orientation.y,
            self.current_odom.pose.pose.orientation.z,
            self.current_odom.pose.pose.orientation.w,
        )
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        return yaw

    def follow_the_wall(self):
        d = self.safe_distance

        # Calculate start-goal line parameters if not already calculated
        if not self.start_goal_line_calculated:
            self.start_goal_line_xstart, self.start_goal_line_ystart = self.start_pose
            self.start_goal_line_xgoal, self.start_goal_line_ygoal = self.target_pose
            
            if self.start_goal_line_xstart != self.start_goal_line_xgoal:
                self.start_goal_line_slope_m = (self.start_goal_line_ygoal - self.start_goal_line_ystart) / (self.start_goal_line_xgoal - self.start_goal_line_xstart)
                self.start_goal_line_y_intercept = self.start_goal_line_ystart - self.start_goal_line_slope_m * self.start_goal_line_xstart
            else:
                # Handle vertical line case
                self.start_goal_line_slope_m = float('inf')
                self.start_goal_line_y_intercept = self.start_goal_line_xstart
            
            self.start_goal_line_calculated = True

        # Get current position
        current_x = self.current_odom.pose.pose.position.x
        current_y = self.current_odom.pose.pose.position.y

        # Calculate distance to start-goal line
        if self.start_goal_line_slope_m == float('inf'):
            current_distance_to_line = abs(current_x - self.start_goal_line_y_intercept)
        else:
            current_distance_to_line = abs(current_y - self.start_goal_line_slope_m * current_x - self.start_goal_line_y_intercept) / math.sqrt(self.start_goal_line_slope_m ** 2 + 1)

        if current_distance_to_line < self.distance_to_start_goal_line_precision:  # Threshold for being on the start-to-goal line
            # Check if the current position is closer to the goal than the hit point
            hit_point_distance_to_goal = self.euclidean_distance(self.hit_point_x, self.hit_point_y, self.target_pose[0], self.target_pose[1])
            current_distance_to_goal = self.euclidean_distance(current_x, current_y, self.target_pose[0], self.target_pose[1])

            if current_distance_to_goal < hit_point_distance_to_goal:
                # Check if the leave point is far enough from the hit point
                leave_point_to_hit_point_diff = math.sqrt((current_x - self.hit_point_x) ** 2 + (current_y - self.hit_point_y) ** 2)
                if leave_point_to_hit_point_diff >= self.leave_point_to_hit_point_diff:
                    # Record leave point and distance to goal
                    self.leave_point_x = current_x
                    self.leave_point_y = current_y
                    self.distance_to_goal_from_leave_point = current_distance_to_goal

                    # Leave the obstacle and move towards the goal
                    if self.previous_state == 'go_to_intermediate':
                        self.state = 'go_to_intermediate'
                        rospy.loginfo("Leaving obstacle, returning to 'go_to_intermediate' state")
                    elif self.previous_state == 'go_to_goal':
                        self.state = 'go_to_goal'
                        rospy.loginfo("Leaving obstacle, returning to 'go_to_goal' state")
                    elif self.previous_state == 'go_to_start':
                        self.state = 'go_to_start'
                        rospy.loginfo("Leaving obstacle, returning to 'go_to_start' state")
                else:
                    self.state = 'follow_boundary'
            else:
                self.state = 'follow_boundary'
        else:
            self.state = 'follow_boundary'

        # Wall following logic
        if self.state == 'follow_boundary':
            #wall following logic to be implemented here.
            if (self.sonar_data['fleft'] > d and
                self.sonar_data['front_left'] > d and
                self.sonar_data['front'] > d and
                self.sonar_data['front_right'] > d and
                self.sonar_data['fright'] > d):
                # Case 1: No obstacle detected
                #search for wall by turning right.
                self.set_velocity([0.25, -0.5])
            elif (self.sonar_data['fleft'] > d and
                self.sonar_data['front_left'] > d and
                self.sonar_data['front'] > d and
                self.sonar_data['front_right'] > d and
                self.sonar_data['fright'] < d):
                # Case 2: Obstacle detected in right
                if (self.sonar_data['fright'] < d - 0.3): #obstacle too close
                    self.set_velocity([0.5, 0.5])
                else:
                    self.set_velocity([0.5, 0])

            elif (self.sonar_data['fleft'] > d and
                self.sonar_data['front_left'] > d and
                self.sonar_data['front'] > d and
                self.sonar_data['front_right'] < d and
                self.sonar_data['fright'] > d):
                # Case 3: Obstacle detected on front-right
                self.set_velocity([0.5, 0.2])
                
            elif (self.sonar_data['fleft'] > d and
                self.sonar_data['front_left'] > d and
                self.sonar_data['front'] > d and
                self.sonar_data['front_right'] < d and
                self.sonar_data['fright'] < d):
                # Case 4: Obstacle detected on front-right and right
                if (self.sonar_data['fright'] < d - 0.3 or self.sonar_data['front_right'] < d - 0.3): #obstacle too close
                    self.set_velocity([0.5, 0.5])
                else:
                    self.set_velocity([0.5, -0.1])
                

            elif (self.sonar_data['fleft'] > d and
                self.sonar_data['front_left'] > d and
                self.sonar_data['front'] < d and
                self.sonar_data['front_right'] > d and
                self.sonar_data['fright'] > d):
                # Case 5: Obstacle detected on front 
                self.set_velocity([0, 0.5])

            elif (self.sonar_data['fleft'] > d and
                self.sonar_data['front_left'] > d and
                self.sonar_data['front'] < d and
                self.sonar_data['front_right'] > d and
                self.sonar_data['fright'] < d):
                # Case 6: Obstacle detected on front and right
                pass

            elif (self.sonar_data['fleft'] > d and
                self.sonar_data['front_left'] > d and
                self.sonar_data['front'] < d and
                self.sonar_data['front_right'] < d and
                self.sonar_data['fright'] > d):
                # Case 7: Obstacle detected on front-right and front
                self.set_velocity([0.3, 0.5])
                
            elif (self.sonar_data['fleft'] > d and
                self.sonar_data['front_left'] > d and
                self.sonar_data['front'] < d and
                self.sonar_data['front_right'] < d and
                self.sonar_data['fright'] < d):
                # Case 8: Obstacle detected on front-right and right and front
                self.set_velocity([0.1, 0.6])
                
            elif (self.sonar_data['fleft'] > d and
                self.sonar_data['front_left'] < d and
                self.sonar_data['front'] > d and
                self.sonar_data['front_right'] > d and
                self.sonar_data['fright'] > d):
                # Case 9: Obstacle detected on front-left 
                self.set_velocity([0.5, -0.1])

            elif (self.sonar_data['fleft'] > d and
                self.sonar_data['front_left'] < d and
                self.sonar_data['front'] > d and
                self.sonar_data['front_right'] > d and
                self.sonar_data['fright'] < d):
                # Case 10: Obstacle detected on left and right
                self.set_velocity([0.5, 0.0])

            elif (self.sonar_data['fleft'] > d and
                self.sonar_data['front_left'] < d and
                self.sonar_data['front'] > d and
                self.sonar_data['front_right'] < d and
                self.sonar_data['fright'] > d):
                # Case 11: Obstacle detected on left, and right
                self.set_velocity([0.5, 0])
                
            elif (self.sonar_data['fleft'] > d and
                self.sonar_data['front_left'] < d and
                self.sonar_data['front'] > d and
                self.sonar_data['front_right'] < d and
                self.sonar_data['fright'] < d):
                # Case 12: Obstacle detected on fright, front-left, and front_right
                self.set_velocity([0.5, 0])
            
            elif (self.sonar_data['fleft'] > d and
                self.sonar_data['front_left'] < d and
                self.sonar_data['front'] < d and
                self.sonar_data['front_right'] > d and
                self.sonar_data['fright'] > d):
                # Case 13: Obstacle detected on front, front_left
                self.set_velocity([0.1, 0.5])

            elif (self.sonar_data['fleft'] > d and
                self.sonar_data['front_left'] < d and
                self.sonar_data['front'] < d and
                self.sonar_data['front_right'] > d and
                self.sonar_data['fright'] < d):
                # Case 14: Obstacle detected on front, front-left, and right
                self.set_velocity([0.2, -0.5])

            elif (self.sonar_data['fleft'] > d and
                self.sonar_data['front_left'] < d and
                self.sonar_data['front'] < d and
                self.sonar_data['front_right'] < d and
                self.sonar_data['fright'] > d):
                # Case 15: Obstacle detected on front, front-left,front_right
                self.set_velocity([0.1, 0.5])

            elif (self.sonar_data['fleft'] > d and
                self.sonar_data['front_left'] < d and
                self.sonar_data['front'] < d and
                self.sonar_data['front_right'] < d and
                self.sonar_data['fright'] < d):
                # Case 16: Obstacle detected on front, front_left, right, and front-right
                self.set_velocity([0.1, 0.6])
                
            elif (self.sonar_data['fleft'] < d and
                self.sonar_data['front_left'] > d and
                self.sonar_data['front'] > d and
                self.sonar_data['front_right'] > d and
                self.sonar_data['fright'] > d):
                # Case 17: Obstacle detected on left
                self.set_velocity([0.5, 0])

            elif (self.sonar_data['fleft'] < d and
                self.sonar_data['front_left'] > d and
                self.sonar_data['front'] > d and
                self.sonar_data['front_right'] > d and
                self.sonar_data['fright'] < d):
                # Case 18: Obstacle detected on left, right
                self.set_velocity([0.5, 0])
                
            elif (self.sonar_data['fleft'] < d and
                self.sonar_data['front_left'] > d and
                self.sonar_data['front'] > d and
                self.sonar_data['front_right'] < d and
                self.sonar_data['fright'] > d):
                # Case 19: Obstacle detected on left, front-right
                self.set_velocity([0.5, 0])
                
            elif (self.sonar_data['fleft'] < d and
                self.sonar_data['front_left'] > d and
                self.sonar_data['front'] > d and
                self.sonar_data['front_right'] < d and
                self.sonar_data['fright'] < d):
                # Case 20: Obstacle detected on left, right, and front-right
                self.set_velocity([0.5, 0])
                
            elif (self.sonar_data['fleft'] < d and
                self.sonar_data['front_left'] > d and
                self.sonar_data['front'] < d and
                self.sonar_data['front_right'] > d and
                self.sonar_data['fright'] > d):
                # Case 21: Obstacle detected on left, front
                self.set_velocity([0.1, 0.5])
                
            elif (self.sonar_data['fleft'] < d and
                self.sonar_data['front_left'] > d and
                self.sonar_data['front'] < d and
                self.sonar_data['front_right'] > d and
                self.sonar_data['fright'] < d):
                # Case 22: Obstacle detected on front,  left, and right
                self.set_velocity([0.1, 0.5])
                
            elif (self.sonar_data['fleft'] < d and
                self.sonar_data['front_left'] > d and
                self.sonar_data['front'] < d and
                self.sonar_data['front_right'] < d and
                self.sonar_data['fright'] < d):
                # Case 23: Obstacle detected on front, front-right, right, and left
                self.set_velocity([0.1, 0.5])
                
            elif (self.sonar_data['fleft'] < d and
                self.sonar_data['front_left'] < d and
                self.sonar_data['front'] > d and
                self.sonar_data['front_right'] > d and
                self.sonar_data['fright'] > d):
                # Case 24: Obstacle detected on left, front-left
                self.set_velocity([0.5, 0])

            elif (self.sonar_data['fleft'] < d and
                self.sonar_data['front_left'] < d and
                self.sonar_data['front'] > d and
                self.sonar_data['front_right'] > d and
                self.sonar_data['fright'] < d):
                # Case 25: Obstacle detected on  left, right, and front-left
                self.set_velocity([0.5, 0])
                
            elif (self.sonar_data['fleft'] < d and
                self.sonar_data['front_left'] < d and
                self.sonar_data['front'] > d and
                self.sonar_data['front_right'] < d and
                self.sonar_data['fright'] > d):
                # Case 26: Obstacle detected on  left, front-right, and front-left
                self.set_velocity([0.5, 0])
                
            elif (self.sonar_data['fleft'] < d and
                self.sonar_data['front_left'] < d and
                self.sonar_data['front'] > d and
                self.sonar_data['front_right'] < d and
                self.sonar_data['fright'] < d):
                # Case 27: Obstacle detected on  left, right, front-left, and front-right
                self.set_velocity([0.5, 0])

            elif (self.sonar_data['fleft'] < d and
                self.sonar_data['front_left'] < d and
                self.sonar_data['front'] < d and
                self.sonar_data['front_right'] > d and
                self.sonar_data['fright'] > d):
                # Case 28: Obstacle detected on front, left, front-left
                self.set_velocity([0.1, 0.5])

            elif (self.sonar_data['fleft'] < d and
                self.sonar_data['front_left'] < d and
                self.sonar_data['front'] < d and
                self.sonar_data['front_right'] > d and
                self.sonar_data['fright'] < d):
                # Case 29: Obstacle detected on front, left, right, front-left
                self.set_velocity([0.1, 0.5])

            elif (self.sonar_data['fleft'] < d and
                self.sonar_data['front_left'] < d and
                self.sonar_data['front'] < d and
                self.sonar_data['front_right'] < d and
                self.sonar_data['fright'] > d):
                # Case 30: Obstacle detected on front, left, front-left, and front-right
                self.set_velocity([0.1, 0.5])

            elif (self.sonar_data['fleft'] < d and
                self.sonar_data['front_left'] < d and
                self.sonar_data['front'] < d and
                self.sonar_data['front_right'] < d and
                self.sonar_data['fright'] < d):
                # Case 31: Obstacle detected on front, left, right, front-left, and front-right
                self.set_velocity([-0.1, 0.5])

            else:
                # Case 32: Unknown scenario
                rospy.logwarn('Unknown case')


    def set_velocity(self, vel):
        twist_msg = Twist()
        twist_msg.linear.x = vel[0]
        twist_msg.angular.z = vel[1]
        self.cmd_vel_pub.publish(twist_msg)

    def stop_robot(self):
        twist_cmd = Twist()
        self.cmd_vel_pub.publish(twist_cmd)
        
if __name__ == '__main__':
    try:
        bug2 = Bug2Algorithm()
        bug2.run()
    except rospy.ROSInterruptException:
        pass