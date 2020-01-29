//
// read gmap map and transforms
// publish map and index
//

#include <ros/console.h>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO_STREAM("map!");
}

int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Starting!");
    ros::init(argc, argv, "nam_node");
    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("/map", 10, &mapCallback);

    ros::spin();
    return 0;
}