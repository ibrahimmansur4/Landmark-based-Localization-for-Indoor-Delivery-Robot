#!/usr/bin/env python3

import rospy
import heapq
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class Node:
    def __init__(self, x, y, cost, heuristic, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.parent = parent

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)

def astar(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Possible movements (right, down, left, up)

    open_set = []
    closed_set = set()

    heapq.heappush(open_set, Node(start[0], start[1], 0, heuristic(start, goal)))

    while open_set:
        current = heapq.heappop(open_set)

        if (current.x, current.y) == goal:
            return reconstruct_path(current)

        closed_set.add((current.x, current.y))

        for dx, dy in directions:
            neighbor_x, neighbor_y = current.x + dx, current.y + dy

            if 0 <= neighbor_x < rows and 0 <= neighbor_y < cols and grid[neighbor_x][neighbor_y] == 0:
                if (neighbor_x, neighbor_y) not in closed_set:
                    neighbor = Node(neighbor_x, neighbor_y, current.cost + 1, heuristic((neighbor_x, neighbor_y), goal), current)
                    heapq.heappush(open_set, neighbor)

    return None  # No path found

def heuristic(point, goal):
    return abs(point[0] - goal[0]) + abs(point[1] - goal[1])

def reconstruct_path(node):
    path = [(node.x, node.y)]

    while node.parent is not None:
        node = node.parent
        path.append((node.x, node.y))

    return path[::-1]

def map_callback(msg):
    global grid, map_width, map_height
    map_width = msg.info.width
    map_height = msg.info.height
    data = msg.data

    # Convert data to a 2D grid
    grid = [[0] * map_width for _ in range(map_height)]

    for i in range(map_height):
        for j in range(map_width):
            index = i * map_width + j
            grid[i][j] = 1 if data[index] > 50 else 0  # Adjust threshold as needed

def publish_path(path):
    path_msg = Path()

    for point in path:
        pose = PoseStamped()
        pose.pose.position.x = point[0] * 0.05 - 100.0  # Convert grid coordinates to world coordinates
        pose.pose.position.y = point[1] * 0.05 - 100.0
        path_msg.poses.append(pose)

    path_msg.header.frame_id = "map"
    path_msg.header.stamp = rospy.Time.now()

    path_publisher.publish(path_msg)

if __name__ == "__main__":
    rospy.init_node("astar_node")

    # Subscribe to the map topic
    rospy.Subscriber("/map", OccupancyGrid, map_callback)

    # Wait for the map to be received
    rospy.wait_for_message("/map", OccupancyGrid)

    # Define start and goal points
    start = (0, 0)
    goal = (50, 50)

    # Plan path using A* algorithm
    path = astar(grid, start, goal)

    if path:
        print("Path found:", path)
        # Publish the path for visualization
        path_publisher = rospy.Publisher("/path", Path, queue_size=10)
        publish_path(path)
    else:
        print("No path found.")

    rospy.spin()
