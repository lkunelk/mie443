// PLAN B CODE
// Idea: Robot SCANS for direction of greatest distance. Then it EXPLORES in that direction.
//      ^ Two phases: Scanning (where the robot rotates) and Exploring (where the robot moves)

#include "Movement.cpp"
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <chrono>
#include <math.h>

// Angle conversions
#define RAD2DEG(rad) ((rad)*180. / M_PI)
#define DEG2RAD(deg) ((deg)*M_PI / 180.)

// General Speed Constants
#define  ROT_SPEED 1

// Scanning constants
#define  SCAN_ROT_SPEED 0.5

// Exploration constants
#define  NUM_SECTORS 12 // Number of sectors to use to find the direction of greatest distance
#define  EXPLORE_SPEED 0.1 // Speed of the forward exploration in m/s
#define  EXPLORE_STEP_SIZE 0.1 // Size of each step forward in meters
#define  CLOSE_THRESH 100 // How close to the obstacle in front before scanning again
#define  SWEEP_INTERVAL 10 // Sweeps left right after certain number of steps forward
#define  SWEEP_RADIUS (M_PI / 6) // How much to sweep side to side

// Operational variables
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
    double furthest_distance = -1;
    double angle_of_furthest_distance = -1;
    
    double rotation;
    double curr_angle;

    for(int i = 1; i <= num_sectors; i++) {
        move.rotate(DEG2RAD(360) / num_sectors, SCAN_ROT_SPEED);
        curr_angle = i * DEG2RAD(360) / num_sectors;
        ros::spinOnce();
        //ROS_INFO("Current dist: %f, Furthest dist: %f", minLaserDist, furthest_distance);

        if(minLaserDist > furthest_distance && minLaserDist < 1000 
            && !(RAD2DEG(curr_angle) < 200 && RAD2DEG(curr_angle) > 160))   { 
            //Second condition to ignore inf. 
            //Third condition to avoid going back the way it came.
            
            furthest_distance = minLaserDist;
            angle_of_furthest_distance = curr_angle;
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
    Move move(nh);

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    // Other variables
    //ros::Rate loop_rate(10);
    double angle;
    int steps;

    // TurtleBot runs until timer runs out
    while (ros::ok() && secondsElapsed <= 480)
    {
        // Phase 1: Scanning
        angle = get_angle_of_furthest_distance(NUM_SECTORS, move);
        move.rotate(angle, ROT_SPEED);

        // Phase 2: Exploring
        steps = 0;
        ros::spinOnce();
        while(minLaserDist < CLOSE_THRESH){
            move.forward(EXPLORE_STEP_SIZE, EXPLORE_SPEED);
            // Sweeping along the way, like a broom
            // if (steps % SWEEP_INTERVAL == 0){
            //     move.rotate(SWEEP_RADIUS, ROT_SPEED);
            //     move.rotate(-2 * SWEEP_RADIUS, ROT_SPEED);
            //     move.rotate(SWEEP_RADIUS, ROT_SPEED);
            // }

            // The last thing to do is to update the timer.
            secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
            if (secondsElapsed > 480 || !ros::ok()){
                break;
            }
            steps++;
            ros::spinOnce();
        }
    }

    return 0;
}
