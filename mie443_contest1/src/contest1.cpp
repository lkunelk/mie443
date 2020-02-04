// THIS CODE IS NOT CONTEST CODE. It's just shows how to use Movement.h
#include "Movement.cpp"
#include "ros/ros.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
 
    Move move(nh);
    while (ros::ok())
    {
        ros::spinOnce();

        /// Logic goes here
        move.rotate(3.14/2, 1);
        move.forward(-1, 0.5);
        move.stop();
    }

    return 0;
}
