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
    ros::Publisher pub = nh.advertise<geometry_msgs::Point>("world_xy", 1000);

    tf::TransformListener listener;
    tf::StampedTransform robotPose;
    while(ros::ok()) {
        try {
            listener.lookupTransform("map", "base_link", ros::Time(0), robotPose);
            ROS_INFO_STREAM("robot pose! " << robotPose.getOrigin().x() << ", " << robotPose.getOrigin().y() << "\n");

            float origin_to_robot_x = robotPose.getOrigin().x() - origin_x;
            float origin_to_robot_y = robotPose.getOrigin().y() - origin_y;
            float grid_x = origin_to_robot_x / resolution;
            float grid_y = origin_to_robot_y / resolution;

            geometry_msgs::Point msg;
            msg.x = grid_x;
            msg.y = grid_y;
            msg.z = tf::getYaw(robotPose.getRotation());//rotation
            pub.publish(msg);

        } catch(tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        ros::spinOnce();
    }

    return 0;
}