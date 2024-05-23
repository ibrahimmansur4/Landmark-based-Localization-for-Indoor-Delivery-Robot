#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <cmath>
#include <vector>
#include <algorithm>

#define SONAR_NUM 8
int offset[SONAR_NUM] = {160, 220, 240, 240, 240, 240, 220, 160};
int distToObstacle[SONAR_NUM] = {0};

class WallFollower {
public:
    WallFollower() : nh_(""), linear_speed_(0.05), angular_speed_(0.1), safe_distance_(500)
    {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 10);
        sonar_sub_ = nh_.subscribe("/RosAria/sonar", 10, &WallFollower::sonarCallback, this);
    }

    void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& sonar_data)
    {
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

    void stopAndRotateLeft()
    {
        // Stop the robot
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        cmd_vel_pub_.publish(twist_msg);

        // Rotate the robot left
        twist_msg.angular.z = angular_speed_;
        cmd_vel_pub_.publish(twist_msg);
    }

    void adjustPosition(const std::vector<int>& right_distances)
    {
        // Check if the right-side sensors detect that the robot is too far from the wall
        if (*std::max_element(right_distances.begin(), right_distances.end()) > safe_distance_ ) {
            // Adjust the angular velocity to move closer to the wall
            geometry_msgs::Twist twist_msg;
            twist_msg.linear.x = linear_speed_;
            twist_msg.angular.z = -angular_speed_;
            cmd_vel_pub_.publish(twist_msg);
        } else if (*std::min_element(right_distances.begin(), right_distances.end()) < safe_distance_ + 100) {
            // Adjust the angular velocity to move away from the wall
            geometry_msgs::Twist twist_msg;
            twist_msg.linear.x = linear_speed_;
            twist_msg.angular.z = angular_speed_;
            cmd_vel_pub_.publish(twist_msg);
        } else {
            // Keep moving forward
            geometry_msgs::Twist twist_msg;
            twist_msg.linear.x = linear_speed_;
            twist_msg.angular.z = 0.0;
            cmd_vel_pub_.publish(twist_msg);
        }
    }

    void run()
    {
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber sonar_sub_;
    double linear_speed_;
    double angular_speed_;
    int safe_distance_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_follower");
    WallFollower wall_follower;
    wall_follower.run();

    return 0;
}
