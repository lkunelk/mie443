// PLAN B CODE
// Idea: Robot SCANS for direction of greatest distance. Then it TRAVELS in that direction.
//      ^ Two States: Scanning (where the robot rotates) and Traveling (where the robot moves)

// Launching simulation bot: roslaunch mie443_contest1 turtlebot_world.launch world:=1
// Gmapping in Sim: roslaunch turtlebot_gazebo gmapping_demo.launch
// Visualizing the Gmap: roslaunch turtlebot_rviz_launchers view_navigation.launch
// Running the code: rosrun mie443_contest1 contest1

// Launching real bot: roslaunch turtlebot_bringup minimal.launch
// Gmapping: roslaunch mie443_contest1 gmapping.launch
// Visualizing the Gmap: roslaunch turtlebot_rviz_launchers view_navigation.launch
// Saving the map: rosrun map_server map_saver -f /home/turtlebot
// Running the code: rosrun mie443_contest1 contest1

// Connecting: ssh tuesday@100.65.103.84

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
    int steps;
    bool ignore_back = false;

    // TurtleBot runs until timer runs out
    while (ros::ok() && secondsElapsed <= 480)
    {
        // State 1: Scanning
        ROS_INFO("SCANNING");
        angle = full_scan(NUM_SECTORS, move, pick_line, ignore_back);
        ignore_back = true;
 
        move.rotate(angle, ROT_SPEED, true);

        // State 2: Travelling
        ROS_INFO("TRAVELLING");
        steps = 0;
        ros::spinOnce();
        while(minLaserDist > CLOSE_THRESH){
            move.forward(EXPLORE_STEP_SIZE, EXPLORE_SPEED);
            // Sweeping along the way, like a broom
            // if (steps % SWEEP_INTERVAL == 0){
            //     move.rotate(SWEEP_RADIUS, ROT_SPEED);
            //     move.rotate(-2 * SWEEP_RADIUS, ROT_SPEED);
            //     move.rotate(SWEEP_RADIUS, ROT_SPEED);
            // }
            if (steps % 10){
                break
            }

            // The last thing to do is to update the timer.
            secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
            if (secondsElapsed > 480 || !ros::ok()){
                break;
            }
            else if(move.is_bumped()){
                // Move backward. Hopefully won't bump into something while going forward
                ROS_WARN("Moving backwards away from collision zone.");
                move.forward(-2 * EXPLORE_STEP_SIZE, EXPLORE_SPEED);
                move.reset_bumped();             
                ignore_back = false;
                ROS_INFO("Escaped from collision zone. Resuming operation.");
                break; // After escaping, it goes back to the scanning state
            }
            steps++;
            ros::spinOnce();

        }
    }
    ROS_INFO("RUN COMPLETE.");

    return 0;
}

// void move_to_point(double x1, double y1, double theta1, double x2, double y2){
//     double distance = ((x2 - x1)^2+(y2 - y1)^2)^0.5;
//     double angle = DEG2RAD(atan((y2 - y1)/(x2 - x1));
//     double rotation;
//     if(angle - yaw > 180) {
//         rotation = -(360-(angle - theta1));
//     }
//     else if(angle - yaw < -180) {
//         rotation = -360 - (angle - theta1);
//     }
//     else {
//         rotation = angle - theta1;
//     }
// }

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

