/*
PLAN B CODE (apparently Plan A as of Feb 6)
=================================================
Overview
Idea: Robot SCANS for direction of greatest distance. Then it TRAVELS in that direction.
So the robot will be a finite state machine:
    States: 
        Scanning: Robot rotates around to find direction to travel.
        Traveling: Robot moves in straight line.
        Collision: Robot dealing with collision: robot stops immediately then moves backwards
    Transitions:
        Scanning -> Traveling: When robot decides on a direction to travel.
        Traveling -> Scanning: When robot reaches end of the path.
        Traveling -> Collision: When robot bumpers is pressed.
        Collision -> Scanning: When robot finishes moving backward
=================================================
Some useful commands.

SIMULATION ------------------------
Launching simulation bot
    roslaunch mie443_contest1 turtlebot_world.launch world:=1
Gmapping in Sim
    roslaunch turtlebot_gazebo gmapping_demo.launch
Visualizing the Gmap
    roslaunch turtlebot_rviz_launchers view_navigation.launch
Running the code
    rosrun mie443_contest1 contest1

REAL LIFE ---------------------------
Remote connecting
    ssh tuesday@100.65.103.84
Launching real bot
    roslaunch turtlebot_bringup minimal.launch
Gmapping
    roslaunch mie443_contest1 gmapping.launch
Visualizing the Gmap
    roslaunch turtlebot_rviz_launchers view_navigation.launch
Saving the map
    rosrun map_server map_saver -f /home/turtlebot
    rosrun map_server map_saver -f ./map

Running the code
    rosrun mie443_contest1 contest1 
*/

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <chrono>
#include <math.h>

#include "Movement.cpp"

// Angle conversions
#define RAD2DEG(rad) ((rad)*180. / M_PI)
#define DEG2RAD(deg) ((deg)*M_PI / 180.)

// General Speed Constants
#define  ROT_SPEED 1 // Normal rotation speed (when not looking for directions)

// Scanning Constants
#define  SCAN_ROT_SPEED 0.4 // Rotation speed when scanning
#define  RADIUS_BACKWARD 75 // Deviation (degrees) from 180 degrees that is considered back of bot

// Traveling constants
#define  NUM_SECTORS 18 // Number of sectors to use to find the direction of greatest distance
#define  TRAVEL_SPEED 0.25 // Speed of the forward exploration in m/s
#define  TRAVEL_SPEED_SLOW 0.1 // Speed of travel when close to obstacles in m/s
#define  TRAVEL_STEP_SIZE 0.1 // Size of each step forward in meters 
#define  TRAVEL_STOP_THRESH 0.55 // How close to the obstacle in front before scanning again (m)
#define  TRAVEL_SLOW_THRESH 0.75 // How close to the obstacle in front before slowing down (m)

// Operational variables
float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers = 0, desiredNLasers = 0, desiredAngle = 5;

// Exploration variables
double angle_of_interest; // Angle found in the Scanning state to be travelled in the Travelling state
int steps; // Stores the number of steps travelled in the Travelling state
bool ignore_back = false; // Whether or not to exclude the angles behind the bot as angle of interest

// Function Declarations
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
double pick_line(int index, int num_sectors, double *distances);
double pick_sector(int index, int num_sectors, double *distances);
double full_scan(int num_sectors, Move move, double (*pick_method)(int, int, double*), bool ignore_back);
double forward_scan(int num_sectors, Move move, double (*pick_method)(int, int, double*), bool _);

// Function Definitions
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

    // TurtleBot runs until timer runs out
    double backup_coeff;
    while (ros::ok() && secondsElapsed <= 480)
    {
        backup_coeff = 1.5;
        // State 1: Scanning ------------------------------
        ROS_INFO("SCANNING");
        angle_of_interest = full_scan(NUM_SECTORS, move, pick_line, ignore_back);
        ignore_back = true; // ignore_back is false only at the beginning and after a collision

        move.rotate(angle_of_interest, ROT_SPEED, true);

        // State 2: Travelling ------------------------------
        ROS_INFO("TRAVELLING");
        steps = 1;
        ros::spinOnce(); // To update minLaserDist
        while(minLaserDist > TRAVEL_STOP_THRESH){
            // Periodic sweeping (not scanning)
            if (steps % 20 == 0){
                ROS_INFO("Periodic Sweeping");
                move.rotate(DEG2RAD(360), ROT_SPEED, true);
            }
            
            // If collide during the loop inside the forward function, that loop will break and
            //  the collision will be handled below.
            ros::spinOnce();
            if (minLaserDist < TRAVEL_SLOW_THRESH){
                move.forward(TRAVEL_STEP_SIZE, TRAVEL_SPEED_SLOW);
            }
            else{
                move.forward(TRAVEL_STEP_SIZE, TRAVEL_SPEED);
            }

            // Timer update
            secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
            if (secondsElapsed > 480 || !ros::ok()){
                break;
            }
            
            // State 3: Collision ------------------------------
            int bump = move.is_collision();
            if(bump > -1){
                // Move backward. Hopefully won't bump into something while going forward
                ROS_WARN("Moving backwards away from collision zone.");
                move.forward(-backup_coeff * TRAVEL_STEP_SIZE, TRAVEL_SPEED);
                move.reset_bumped();             
                if(bump == kobuki_msgs::BumperEvent::CENTER) {
                    ROS_INFO("Bumped Center");
                    ignore_back = false; // After a collision, the robot should consider moving back the way it came
                } else {
                    ROS_INFO("Bumped Side");
                }

                backup_coeff = std::max(backup_coeff + 0.5, 4.0);
                ROS_INFO("Escaped from collision zone. Resuming operation.");
                break; // After escaping, it goes back to the Scanning state
            }
            steps = steps + 1;
            ros::spinOnce(); // To update minLaserDist (for the while loop condition)
        }
    }
    ROS_INFO("RUN COMPLETE.");

    return 0;
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
    // When looking at the distance of a certain sector (index), takes into account neighbouring sectors

    // Compute the indices of the array to be analyzed 
    int next_i = (index + 1) % num_sectors;
    int prev_i = index - 1;
    if (prev_i < 0){prev_i = num_sectors - 1;}

    // Compute the neighbourhood average and see if it's a candidate direction
    double average_dist = (distances[prev_i] + distances[index] + distances[next_i]) / 3;
    return average_dist;
}

double pick_line(int index, int num_sectors, double *distances){
    // Just gives the distance in the current sector
    return distances[index];
}

double full_scan(int num_sectors, Move move, double(*pick_method)(int, int, double*), bool ignore_back){
    // Returns angle to rotate from current position toward the direction of furthest distance
    // Have a choice of "pick_method". Either use pick_line or pick_sector defined above.

    double distances[num_sectors]; // minLaserDists for each sector
    double furthest_distance = -1; // Stores the current largest distance
    double angle_of_furthest_distance = -1; // Stores the angle that corresponds to the furtheset_distance
    
    double curr_angle; // Angle corresponding to current index
    double dist; // Distance value for a particular sector

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

