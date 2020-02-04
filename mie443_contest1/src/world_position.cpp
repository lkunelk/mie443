//
// Created by nam on 2020-02-04.
//

#include <ros/console.h>
#include "ros/ros.h"
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Starting!");
    ros::init(argc, argv, "world_position");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    tf::StampedTransform robotPose;
    while(ros::ok()) {
        try {
            listener.lookupTransform("map", "base_link", ros::Time(0), robotPose);
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