#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <unordered_map>
#include <string>

const int SONAR_NUM = 16; // Assuming 16 sonar sensors
int distToObstacle[SONAR_NUM];
int offset[SONAR_NUM]; // Offset values for each sonar sensor

class RobotMover {
public:
    RobotMover(double linear_velocity, double angular_velocity, const std::string& goal_room_name)
        : linear_velocity_(linear_velocity),
          angular_velocity_(angular_velocity),
          max_linear_speed_(0.05),
          max_angular_speed_(0.1),
          safe_distance_(500),
          state_("wall_follow"),
          angular_tolerance_(0.1),
          goal_room_name_(goal_room_name) {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 10);
        odom_subscriber_ = nh_.subscribe("/RosAria/odom", 1, &RobotMover::odomCallback, this);
        sonar_subscriber_ = nh_.subscribe("/RosAria/sonar", 10, &RobotMover::sonarCallback, this);
        current_room_sub_ = nh_.subscribe("/current_room", 1, &RobotMover::currentRoomCallback, this);
    }

    void moveAutonomously() {
        ROS_INFO("Starting autonomous exploration. Press Ctrl+C to stop.");
        ros::Rate rate(1);  // 1 Hz rate for state transitions

        while (ros::ok()) {
            if (state_ == "wall_follow") {
                for (int i = 0; i < 3; ++i) {  // Wall follow for 3 seconds
                    ROS_INFO("Wall follow state.");
                    sonarCallback(sonar_data_);
                    ros::Duration(1.0).sleep();
                }
                state_ = "detecting_landmark";
                ROS_INFO("Changed state to detect.");
            } else if (state_ == "detecting_landmark") {
                ROS_INFO("Detect Landmark state.");
                detectingLandmark();
                ros::Duration(1.0).sleep();  // Read landmark for 1 second
            } else if (state_ == "go_to_goal") {
                ROS_INFO("Go to goal state.");
                goToGoal();
            } else if (state_ == "stop") {
                ROS_INFO("Stop state.");
                stop();
            }
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_subscriber_;
    ros::Subscriber sonar_subscriber_;
    ros::Subscriber current_room_sub_;

    double linear_velocity_;
    double angular_velocity_;
    double max_linear_speed_;
    double max_angular_speed_;
    int safe_distance_;
    std::string state_;
    double angular_tolerance_;
    std::string current_room_name_;
    std::string goal_room_name_;

    nav_msgs::Odometry current_odom_;
    sensor_msgs::PointCloud sonar_data_;
    geometry_msgs::Pose previous_pose_;

    void publishTwistCommand(const geometry_msgs::Twist& twist_cmd) {
        cmd_vel_pub_.publish(twist_cmd);
        ros::Duration(0.1).sleep();  // Publish at 10 Hz
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_odom_ = *msg;
    }

    void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& sonar_data) {
        ROS_INFO("Frame[%d]:", sonar_data->header.seq);

        for (int i = 0; i < SONAR_NUM; ++i) {
            double tmpX = sonar_data->points[i].x;
            double tmpY = sonar_data->points[i].y;
            distToObstacle[i] = static_cast<int>(std::sqrt(tmpX * tmpX + tmpY * tmpY) * 1000 - offset[i]);
            ROS_INFO("%d\t", distToObstacle[i]);
        }
        ROS_INFO("\n");

        // Front sensors
        std::vector<int> front_distances(distToObstacle + 3, distToObstacle + 7);

        // Right side sensors
        std::vector<int> right_distances(distToObstacle + 7, distToObstacle + 8);

        // Check if there is an obstacle in the front sensors
        if (*std::min_element(front_distances.begin(), front_distances.end()) < safe_distance_) {
            stopAndRotateLeft();
        } else {
            adjustPosition(right_distances);
        }
    }

    void stopAndRotateLeft() {
        // Stop the robot
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        cmd_vel_pub_.publish(twist_msg);

        // Rotate the robot left
        twist_msg.angular.z = max_angular_speed_;
        cmd_vel_pub_.publish(twist_msg);
    }

    void adjustPosition(const std::vector<int>& right_distances) {
        // Check if the right-side sensors detect that the robot is too far from the wall
        if (*std::max_element(right_distances.begin(), right_distances.end()) > safe_distance_) {
            // Adjust the angular velocity to move closer to the wall
            geometry_msgs::Twist twist_msg;
            twist_msg.linear.x = max_linear_speed_;
            twist_msg.angular.z = -max_angular_speed_;
            cmd_vel_pub_.publish(twist_msg);
        } else if (*std::min_element(right_distances.begin(), right_distances.end()) < safe_distance_ + 100) {
            // Adjust the angular velocity to move away from the wall
            geometry_msgs::Twist twist_msg;
            twist_msg.linear.x = max_linear_speed_;
            twist_msg.angular.z = max_angular_speed_;
            cmd_vel_pub_.publish(twist_msg);
        } else {
            // Keep moving forward
            geometry_msgs::Twist twist_msg;
            twist_msg.linear.x = max_linear_speed_;
            twist_msg.angular.z = 0.0;
            cmd_vel_pub_.publish(twist_msg);
        }
    }

    void currentRoomCallback(const std_msgs::String::ConstPtr& msg) {
        current_room_name_ = msg->data;
    }

    void detectingLandmark() {
        if (!current_room_name_.empty()) {
            if (current_room_name_ == goal_room_name_) {
                state_ = "stop";
                ROS_INFO("Detected goal room %s, stopping.", goal_room_name_.c_str());
            } else {
                state_ = "go_to_goal";
                ROS_INFO("Detected room %s, state set to go_to_goal", current_room_name_.c_str());
            }
        } else {
            state_ = "wall_follow";
            ROS_INFO("No room detected, state set to wall_follow");
        }
    }

void goToGoal() {
    geometry_msgs::Twist twist_msg;
    double start_x, start_y, goal_x, goal_y;

    // Use the current position as the start position
    start_x = current_odom_.pose.pose.position.x;
    start_y = current_odom_.pose.pose.position.y;

    // Get the goal position from the room coordinates
    std::unordered_map<std::string, std::unordered_map<std::string, double>> room_coordinates = {
        {"A-101", {{"x", -2.59}, {"y", -4.31}}},
        {"C-110", {{"x", 2.59}, {"y", -7.71}}},
        {"C-107", {{"x", -2.59}, {"y", 5.60}}}
    };

    auto it = room_coordinates.find(goal_room_name_);
    if (it != room_coordinates.end()) {
        goal_x = it->second["x"];
        goal_y = it->second["y"];
    } else {
        ROS_ERROR("Goal room %s not found in room coordinates.", goal_room_name_.c_str());
        return;
    }

    // Extract the quaternion components and pass them to euler_from_quaternion
    double current_x, current_y, current_yaw;
    current_x = current_odom_.pose.pose.position.x;
    current_y = current_odom_.pose.pose.position.y;

    tf::Quaternion q(
        current_odom_.pose.pose.orientation.x,
        current_odom_.pose.pose.orientation.y,
        current_odom_.pose.pose.orientation.z,
        current_odom_.pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw);

    // Calculate angle to rotate towards the goal
    double goal_angle = std::atan2(goal_y - current_y, goal_x - current_x);
    double angle_difference = goal_angle - current_yaw;

    // Ensure angle is between -pi and pi for proper rotation
    angle_difference = std::fmod(angle_difference + M_PI, 2 * M_PI) - M_PI;

    // Rotate the robot to face the goal
    if (std::abs(angle_difference) > angular_tolerance_) {
        twist_msg.angular.z = (angle_difference > 0) ? angular_velocity_ : -angular_velocity_;
        while (std::abs(angle_difference) > angular_tolerance_ && ros::ok()) {
            cmd_vel_pub_.publish(twist_msg);
            ros::Duration(0.1).sleep();  // Sleep for a short time to allow the robot to rotate

            // Update current_yaw
            current_x = current_odom_.pose.pose.position.x;
            current_y = current_odom_.pose.pose.position.y;
            q = tf::Quaternion(
                current_odom_.pose.pose.orientation.x,
                current_odom_.pose.pose.orientation.y,
                current_odom_.pose.pose.orientation.z,
                current_odom_.pose.pose.orientation.w
            );
            m.setRotation(q);
            m.getRPY(roll, pitch, current_yaw);

            angle_difference = goal_angle - current_yaw;
            angle_difference = std::fmod(angle_difference + M_PI, 2 * M_PI) - M_PI;
        }
    }

    // Stop rotating once facing the goal
    twist_msg.angular.z = 0.0;
    cmd_vel_pub_.publish(twist_msg);

    // Calculate distance to goal and time to reach goal based on robot's speed
    double current_distance = std::sqrt(std::pow(current_x - goal_x, 2) + std::pow(current_y - goal_y, 2));
    double time_to_goal = current_distance / linear_velocity_;
    ROS_INFO("Time to goal: %f", time_to_goal);

    twist_msg.linear.x = linear_velocity_;

    // Start a timer
    ros::Time start_time = ros::Time::now();
    ros::Duration time_moved(0.0);
    ros::Duration remaining_time(time_to_goal);

    while (remaining_time.toSec() > 0.0 && ros::ok()) {
        if (sonar_data_.points.size() > 0 && sonar_data_.points[0].x < 0.9) {  // Obstacle detected
            // Save the time moved and calculate remaining time
            time_moved = ros::Time::now() - start_time;
            remaining_time = ros::Duration(time_to_goal) - time_moved;
            ROS_INFO("Remaining time: %f", remaining_time.toSec());

            if (time_moved.toSec() < 0.7 * time_to_goal) {  // Only handle obstacle if less than 3/4 of the total time has elapsed
                twist_msg.linear.x = 0.0;  // Stop the robot
                cmd_vel_pub_.publish(twist_msg);
                ROS_WARN("Obstacle detected, stopping robot.");

                // Wait for the obstacle to be removed
                while (sonar_data_.points.size() > 0 && sonar_data_.points[0].x < 0.5 && ros::ok()) {
                    ros::Duration(0.1).sleep();
                }

                ROS_INFO("Obstacle removed, continuing to goal.");
                start_time = ros::Time::now();  // Reset start time
                twist_msg.linear.x = linear_velocity_;  // Start moving again
            } else {
                // Stop the robot completely
                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = 0.0;  // Set angular velocity to 0
                cmd_vel_pub_.publish(twist_msg);

                // Wait until the robot has stopped
                while (!isRobotStopped(current_odom_) && ros::ok()) {
                    ros::Duration(0.1).sleep();
                    cmd_vel_pub_.publish(twist_msg);
                }

                state_ = "wall_follow";
                ROS_WARN("Wall detected, starting wall follow.");
                break;
            }
        }

        cmd_vel_pub_.publish(twist_msg);
        ros::Duration(0.1).sleep();  // Sleep for a short time to allow the robot to move
    }

    // Stop the robot if it reached the goal or if an obstacle was detected and handled
    if (state_ != "stop" && state_ != "wall_follow") {
        state_ = "stop";
        ROS_INFO("Reached goal position, stopping.");
        twist_msg.linear.x = 0.0;
        cmd_vel_pub_.publish(twist_msg);
    }
}

bool isRobotStopped(const nav_msgs::Odometry& odom) {
    double linear_velocity = std::sqrt(
        std::pow(odom.twist.twist.linear.x, 2) +
        std::pow(odom.twist.twist.linear.y, 2) +
        std::pow(odom.twist.twist.linear.z, 2)
    );
    double angular_velocity = std::sqrt(
        std::pow(odom.twist.twist.angular.x, 2) +
        std::pow(odom.twist.twist.angular.y, 2) +
        std::pow(odom.twist.twist.angular.z, 2)
    );
    return (linear_velocity < 0.01 && angular_velocity < 0.01);
}

void stop() {
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    cmd_vel_pub_.publish(twist_msg);
    ROS_INFO("Robot stopped.");
}

void wallFollow() {
    geometry_msgs::Twist twist_msg;
    double wall_distance = 0.5;  // Desired distance from the wall
    double angular_velocity_gain = 0.5;  // Gain for angular velocity
    double lateral_distance;

    while (state_ == "wall_follow" && ros::ok()) {
        // Calculate the lateral distance to the wall
        lateral_distance = sonar_data_.points[0].x * std::cos(sonar_data_.points[0].y);

        // Adjust angular velocity based on the lateral distance error
        double angular_velocity = (lateral_distance - wall_distance) * angular_velocity_gain;
        twist_msg.angular.z = angular_velocity;

        // Move forward with a constant linear velocity
        twist_msg.linear.x = linear_velocity_;

        cmd_vel_pub_.publish(twist_msg);
        ros::Duration(0.1).sleep();  // Sleep for a short time to allow the robot to move

        // Check if the robot has reached the goal room
        double current_x = current_odom_.pose.pose.position.x;
        double current_y = current_odom_.pose.pose.position.y;
        std::unordered_map<std::string, std::unordered_map<std::string, double>> room_coordinates = {
            {"A-101", {{"x", -2.59}, {"y", -4.31}}},
            {"C-110", {{"x", 2.59}, {"y", -7.71}}},
            {"C-107", {{"x", -2.59}, {"y", 5.60}}}
        };

        auto it = room_coordinates.find(goal_room_name_);
        if (it != room_coordinates.end()) {
            double goal_x = it->second["x"];
            double goal_y = it->second["y"];
            double distance_to_goal = std::sqrt(std::pow(current_x - goal_x, 2) + std::pow(current_y - goal_y, 2));

            if (distance_to_goal < 0.5) {  // Reached the goal room
                state_ = "stop";
                ROS_INFO("Reached goal room %s, stopping.", goal_room_name_.c_str());
                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = 0.0;
                cmd_vel_pub_.publish(twist_msg);
                break;
            }
        }
    }
}
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_mover");
    ros::NodeHandle nh;

    std::string goal_room_name;
    if (!nh.getParam("/goal_room_name", goal_room_name)) {
        ROS_ERROR("Failed to get goal room name parameter.");
        return 1;
    }

    double linear_velocity, angular_velocity;
    nh.param("linear_velocity", linear_velocity, 0.2);
    nh.param("angular_velocity", angular_velocity, 0.5);

    RobotMover robot_mover(linear_velocity, angular_velocity, goal_room_name);
    robot_mover.moveAutonomously();

    return 0;
}