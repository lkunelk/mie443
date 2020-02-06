//
// read gmap map and transforms
// publish map and index
//

#include <ros/console.h>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>

int origin_x, origin_y;
int height, width;
float resolution;
int robot_x, robot_y;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    geometry_msgs::Point p = msg->info.origin.position;
    height = msg->info.width;
    width = msg->info.height;
    resolution = msg->info.resolution;
    origin_x = p.x;
    origin_y = p.y;
    ROS_INFO_STREAM("map_origin");
}

int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Starting!");
    ros::init(argc, argv, "nam_node");
    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("/map", 10, &mapCallback);

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

    return 0;
}