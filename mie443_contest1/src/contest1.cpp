// THIS CODE IS NOT CONTEST CODE. It's just shows how to use Movement.h
#include "Movement.cpp"
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <chrono>
#include <math.h>

#define RAD2DEG(rad) ((rad)*180. / M_PI)
#define DEG2RAD(deg) ((deg)*M_PI / 180.)

#define  CLOSE_THRESH 50
#define  SCAN_ROT_SPEED 0.5
#define  ROT_SPEED 1
#define  EXPLORE_SPEED 0.1
#define  EXPLORE_INCREMENTS 0.1

float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers = 0, desiredNLasers = 0, desiredAngle = 5;


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

double get_angle_of_furthest_distance(int num_sectors, Move move){
    //Returns angle to rotate from current position toward the direction of furthest distance
    float furthest_distance = std::numeric_limits<float>::infinity();
    float angle_of_furthest_distance = std::numeric_limits<float>::infinity();
    
    float rotation;

    for(int i = 1; i <= num_sectors; i++) {
        move.rotate(DEG2RAD(360) / num_sectors, SCAN_ROT_SPEED);
        ros::spinOnce();
        //ROS_INFO("Current dist: %f, Furthest dist: %f", minLaserDist, furthest_distance);
        if(minLaserDist > furthest_distance) {
            furthest_distance = minLaserDist;
            angle_of_furthest_distance = i * DEG2RAD(360) / num_sectors;
        }
    }

    if(angle_of_furthest_distance > M_PI){
        rotation = angle_of_furthest_distance - DEG2RAD(360); //e.g 270 degrees -> -90 degrees
    }
    else{
        rotation = angle_of_furthest_distance;
    }

    return rotation;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    Move move(nh, true);

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    // Other variables
    //ros::Rate loop_rate(10);
    double angle;

    while (ros::ok() && secondsElapsed <= 480)
    {
        angle = get_angle_of_furthest_distance(10, move);
        ROS_INFO("Rotating to %f", angle);
        move.rotate(angle, ROT_SPEED);
        ros::spinOnce();
        while(minLaserDist < CLOSE_THRESH){
            move.forward(EXPLORE_INCREMENTS, EXPLORE_SPEED);
            
            // The last thing to do is to update the timer.
            secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
            if (secondsElapsed > 480 || !ros::ok()){
                break;
            }
            ros::spinOnce();
        }
    }

    return 0;
}
