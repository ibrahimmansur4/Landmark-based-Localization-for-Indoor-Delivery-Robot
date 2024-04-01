#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist, Point
from math import sqrt

# Define the grid size and obstacle threshold
GRID_SIZE = 100
OBSTACLE_THRESHOLD = 0.5

# Initialize the grid and obstacle map
grid = np.zeros((GRID_SIZE, GRID_SIZE))
obstacle_map = np.zeros((GRID_SIZE, GRID_SIZE))

# Define the robot's initial and final positions
start_x, start_y = 0, 0
goal_x, goal_y = 2, 5

# Define the robot's position and orientation
robot_x, robot_y, robot_theta = start_x, start_y, 0.0

# Define the callback functions
def sonar_callback(data):
    global obstacle_map
    # Update the obstacle map based on sonar data
    angle = 0.0  # Assuming the sonar sensor directly faces forward
    if data.range < data.max_range:
        r = data.range
        x = int(robot_x + r * np.cos(robot_theta + angle))
        y = int(robot_y + r * np.sin(robot_theta + angle))
        obstacle_map[y, x] = 1





def odom_callback(data):
    global robot_x, robot_y, robot_theta
    # Update the robot's position and orientation
    robot_x = data.pose.pose.position.x
    robot_y = data.pose.pose.position.y
    # Update the robot's orientation (you may need to adjust this based on your setup)
    robot_theta = data.pose.pose.orientation.z

# A* path planning algorithm
def astar(start, goal):
    open_set = set([start])
    closed_set = set()
    g_score = {start: 0}
    f_score = {start: manhattan_distance(start, goal)}
    parents = {}

    while open_set:
        current = min(open_set, key=lambda x: f_score[x])
        if current == goal:
            path = reconstruct_path(parents, current)
            return path

        open_set.remove(current)
        closed_set.add(current)

        for neighbor in get_neighbors(current):
            if neighbor in closed_set:
                continue

            tentative_g_score = g_score[current] + 1
            if neighbor not in open_set or tentative_g_score < g_score[neighbor]:
                parents[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal)
                if neighbor not in open_set:
                    open_set.add(neighbor)

    return None

def manhattan_distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)

def get_neighbors(node):
    x, y = node
    neighbors = []
    for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and obstacle_map[ny, nx] == 0:
            neighbors.append((nx, ny))
    return neighbors

def reconstruct_path(parents, current):
    path = [current]
    while current in parents:
        current = parents[current]
        path.append(current)
    path.reverse()
    return path

# ROS node initialization
rospy.init_node('path_planner')

# Subscribe to the sonar and odometry topics
sonar_sub = rospy.Subscriber('/sonar_scan', Range, sonar_callback)
odom_sub = rospy.Subscriber('/RosAria/odom', Odometry, odom_callback)

# Create a publisher for the velocity command
cmd_vel_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)

# Main loop
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    # Plan the path
    start_grid = (int(start_y), int(start_x))
    goal_grid = (int(goal_y), int(goal_x))
    path = astar(start_grid, goal_grid)

    if path:
        # Follow the path
        for point in path:
            goal_x, goal_y = point
            goal_x = goal_x * GRID_SIZE / 100.0
            goal_y = goal_y * GRID_SIZE / 100.0

            # Move the robot towards the goal
            twist = Twist()
            twist.linear.x = 0.5  # Adjust the linear velocity as needed
            twist.angular.z = 0.0  # Adjust the angular velocity as needed
            cmd_vel_pub.publish(twist)

            rate.sleep()
    else:
        rospy.loginfo("No path found")

    rate.sleep()