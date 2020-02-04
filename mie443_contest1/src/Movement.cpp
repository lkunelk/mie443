#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

class Move{
    private:
        ros::Publisher vel_pub;
        geometry_msgs::Twist vel;
        double next_update;

    public:
    Move(ros::NodeHandle nh){
        vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
        ROS_INFO("Mover initiated.");
    }

    void forward(float distance, float speed){
        next_update = ros::WallTime::now().toSec() + std::abs(distance) / std::abs(speed);
        vel.linear.x = speed * std::abs(distance) / distance;
        vel.angular.z = 0;

        ROS_INFO("Moving %f m, will take %f s, will finish at %f s", distance, std::abs(distance) / std::abs(speed), next_update);
        while(ros::ok() && ros::WallTime::now().toSec() < next_update){
            vel_pub.publish(vel);
        }
    }

    void rotate(float angle, float speed){
        next_update = ros::WallTime::now().toSec() + std::abs(angle) / std::abs(speed);
        vel.linear.x = 0;
        vel.angular.z = speed * std::abs(angle) / angle;

        ROS_INFO("Rotating %f, will take %f s, will finish at %f s", angle, std::abs(angle) / std::abs(speed), next_update);    
        while(ros::ok() && ros::WallTime::now().toSec() < next_update){
            vel_pub.publish(vel);
        }
    }

    void stop(){
        ROS_INFO("STOPPED");
        vel.linear.x = 0;
        vel.angular.z = 0;

        vel_pub.publish(vel);
    }

    double get_next_update(){
        return next_update;
    }
};

