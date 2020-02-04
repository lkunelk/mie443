//
// Created by nam on 2020-02-04.
//

#include <ros/console.h>
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>

int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Starting!");
    ros::init(argc, argv, "world_position");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Point>("world_xy", 1000);

    tf::TransformListener listener;
    tf::StampedTransform robotPose;
    while(ros::ok()) {
        try {
            listener.lookupTransform("map", "base_link", ros::Time(0), robotPose);
            geometry_msgs::Point msg;
            msg.x = robotPose.getOrigin().x();
            msg.y = robotPose.getOrigin().y();
            pub.publish(msg);
            ROS_INFO_STREAM("robot pose! " << robotPose.getOrigin().x() << ", " << robotPose.getOrigin().y() << "\n");
        } catch(tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }
    ros::spin();
    return 0;
}