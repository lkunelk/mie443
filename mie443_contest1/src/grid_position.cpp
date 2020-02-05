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
int robot_x, robot_y;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    geometry_msgs::Point p = msg->info.origin.position;
    origin_x = p.x;
    origin_y = p.y;
    ROS_INFO_STREAM("map_origin");
}

void worldPositionCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    robot_x = msg->x;
    robot_y = msg->y;
    ROS_INFO_STREAM("world_pose");
}

int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Starting!");
    ros::init(argc, argv, "nam_node");
    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("/map", 10, &mapCallback);
    ros::Subscriber world_position_sub = nh.subscribe("world_xy", 10, &worldPositionCallback);
    while(ros::ok()) {
        ROS_INFO_STREAM("world: " << origin_x << ", " << origin_y << " robot: " << robot_x << ", " << robot_y << "\n");
        ros::spinOnce();
    }
    return 0;
}