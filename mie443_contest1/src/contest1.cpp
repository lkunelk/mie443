// THIS CODE IS NOT CONTEST CODE. It's just shows how to use Movement.h
#include "Movement.cpp"
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <chrono>


float minLaserDist = std::numeric_limits<float>::infinity();
float CLOSE_THRESH = 50;
float EXPLORE_SPEED = 0.1;
float EXPLORE_INCREMENTS = 0.1;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle) / msg->angle_increment;
    // ROS_INFO("Size of laser scan array: % i and size of offset: % i ", nLasers, desiredNLasers);

    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min)
    {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx)
        {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
    else
    {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx)
        {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
}

double get_angle_of_furthest_distance(){
    //Returns angle to rotate from current position toward the direction of furthest distance
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    Move move(nh);

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    while (ros::ok() && secondsElapsed <= 480)
    {
        ros::spinOnce();

        angle = get_angle_of_furthest_distance();
        move.rotate(angle);
        while(minLaserDist < CLOSE_THRESH){
            move.forward(EXPLORE_INCREMENTS, EXPLORE_SPEED);
            // The last thing to do is to update the timer.
            secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
        }


`    }

    return 0;
}
