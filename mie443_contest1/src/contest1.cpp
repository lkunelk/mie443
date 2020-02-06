// PLAN B CODE
// Idea: Robot SCANS for direction of greatest distance. Then it TRAVELS in that direction.
//      ^ Two States: Scanning (where the robot rotates) and Traveling (where the robot moves)

// Launching simulation bot: roslaunch mie443_contest1 turtlebot_world.launch world:=1
// Gmapping in Sim: roslaunch turtlebot_gazebo gmapping_demo.launch
// Visualizing the Gmap: roslaunch turtlebot_rviz_launchers view_navigation.launch
// Running the code: rosrun mie443_contest1 contest1

// Launching real bot: roslaunch turtlebot_bringup minimal.launch
// Gmapping: roslaunch mie443_contest1 gmapping.launch <-- HAVE TO RUN THIS to use laser
// Visualizing the Gmap: roslaunch turtlebot_rviz_launchers view_navigation.launch
// Saving the map: rosrun map_server map_saver -f /home/turtlebot
// Running the code: rosrun mie443_contest1 contest1

// To connect: ssh tuesday@100.65.103.84

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <chrono>
#include <math.h>

#include "Movement.cpp"

// Angle conversions
#define RAD2DEG(rad) ((rad)*180. / M_PI)
#define DEG2RAD(deg) ((deg)*M_PI / 180.)

// General Speed Constants
#define  ROT_SPEED 1

// Scanning constants
#define  SCAN_ROT_SPEED 0.4
#define  RADIUS_BACKWARD 75 // Deviation (degrees) from 180 degrees that is considered back 

// Traveling constants
#define  NUM_SECTORS 18 // Number of sectors to use to find the direction of greatest distance
#define  EXPLORE_SPEED 0.25 // Speed of the forward exploration in m/s
#define  EXPLORE_STEP_SIZE 0.1 // Size of each step forward in meters
#define  CLOSE_THRESH 0.55 // How close to the obstacle in front before scanning again
// #define  SWEEP_INTERVAL 10 // Sweeps left right after certain number of steps forward
// #define  SWEEP_RADIUS (M_PI / 6) // How much to sweep side to side

// Operational variables
float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers = 0, desiredNLasers = 0, desiredAngle = 5;

// Function Definitions
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
double pick_line(int index, int num_sectors, double *distances);
double pick_sector(int index, int num_sectors, double *distances);
double full_scan(int num_sectors, Move move, double (*pick_method)(int, int, double*), bool ignore_back);
double forward_scan(int num_sectors, Move move, double (*pick_method)(int, int, double*), bool _);

// Function Declarations
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

    move.rotate(DEG2RAD(360), ROT_SPEED);
}

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

double pick_sector(int index, int num_sectors, double *distances){
    int next_i = (index + 1) % num_sectors;
    int prev_i = index - 1;
    if (prev_i < 0){prev_i = num_sectors - 1;}

    // Compute the neighbourhood average and see if it's a candidate direction
    double average_dist = (distances[prev_i] + distances[index] + distances[next_i]) / 3;
    return average_dist;
}

double pick_line(int index, int num_sectors, double *distances){
    return distances[index];
}

double full_scan(int num_sectors, Move move, double(*pick_method)(int, int, double*), bool ignore_back){
    // Returns angle to rotate from current position toward the direction of furthest distance
    double distances[num_sectors];
    double furthest_distance = -1;
    double angle_of_furthest_distance = -1;
    
    double curr_angle;
    double dist;

    int prev_i, next_i;

    // Get the minLaserDist around the robot
    for(int i = 0; i < num_sectors; i++) {
        move.rotate(DEG2RAD(360) / num_sectors, SCAN_ROT_SPEED);
        ros::spinOnce();
        distances[i] = minLaserDist;
    }

    // Analyze the distances and pick direction to go
    for (int i = 0; i < num_sectors; i++){
        curr_angle = (i + 1) * DEG2RAD(360) / num_sectors;
        
        // If the angle corresponding to the sector is not in the backward section
        if(!(RAD2DEG(curr_angle) < 180 + RADIUS_BACKWARD
             && RAD2DEG(curr_angle) > 180 - RADIUS_BACKWARD) || !ignore_back){
            // Compute the indices of the array to be analyzed 
            dist = (*pick_method)(i, num_sectors, distances);

            if (dist > furthest_distance && dist < 1000){
                furthest_distance = dist;
                angle_of_furthest_distance = curr_angle;
            }
            ROS_INFO("Angle: %f, sees %f", RAD2DEG(curr_angle), dist);
        }
    }
    // Post-processing of the angle
    if (angle_of_furthest_distance == -1){
        // If no angles gave valid distances, go back the way it cam
        angle_of_furthest_distance = DEG2RAD(180); 
    }
    else if(angle_of_furthest_distance > DEG2RAD(180)){
        //e.g 270 degrees -> -90 degrees
        angle_of_furthest_distance = angle_of_furthest_distance - DEG2RAD(360); 
    }

    return angle_of_furthest_distance;
}

double forward_scan(int num_sectors, Move move, double(*pick_method)(int, int, double*), bool _){
    // Returns angle to rotate from current position toward the direction of furthest distance
    // This time, it is based on the surroundings sectors as well.
    // Also, it will only sweep out the required areas.
    // NOTE: The sector number will now divide only the sweeped area, not the entire 360
    double distances[num_sectors];
    double furthest_distance = -1; 
    double angle_of_furthest_distance = -1;
    
    double curr_angle;
    double dist;

    int prev_i, next_i;

    double sweep_angle = DEG2RAD(360 - 2 * RADIUS_BACKWARD);
    double sector_size = sweep_angle / num_sectors;

    // Rotate to start position
    move.rotate(sweep_angle / 2 + sector_size, ROT_SPEED);

    // Rotate back the other way to scan to get the minLaserDist around the robot
    for(int i = 0; i < num_sectors + 2; i++) { // Extra two sectors to account for edges
        move.rotate(sector_size, -SCAN_ROT_SPEED);
        ros::spinOnce();
        distances[i] = minLaserDist; // First and last elements are not considered as candidates
    }

    // Analyze the distances and pick direction to go
    for (int i = 1; i < num_sectors + 1; i++){
        // Calculating the angle of the sector wrt original orientation
        curr_angle = sweep_angle / 2 - i * sector_size;

        // Compute the neighbourhood average and see if it's a candidate direction
        dist = (*pick_sector)(i, num_sectors, distances);
        if (dist > furthest_distance && dist < 1000){
            furthest_distance = dist;

            // Calculate angle needed to get to desired angle
            // Currently robot is located at -(sweep_angle / 2 + sector_size) wrt original orientation
            angle_of_furthest_distance = curr_angle + sweep_angle / 2 + sector_size;
        }
    }
    
    // Post-processing of the angle
    if (angle_of_furthest_distance == -1){
        // If no angles gave valid distances, go back the way it cam
        angle_of_furthest_distance = DEG2RAD(180); 
    }
    else if(angle_of_furthest_distance > DEG2RAD(180)){
        //e.g 270 degrees -> -90 degrees
        angle_of_furthest_distance = angle_of_furthest_distance - DEG2RAD(360); 
    }

    return angle_of_furthest_distance;
}
    