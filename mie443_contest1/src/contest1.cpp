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
#define  EXPLORE_SPEED 0.25 // Speed of the forward exploration in m/s
#define  EXPLORE_STEP_SIZE 0.1 // Size of each step forward in meters 
#define  CLOSE_THRESH 0.55 // How close to the obstacle in front before scanning again (m)

// Function Definitions
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    Move move(nh);

    //move.rotate(DEG2RAD(2*360), SCAN_ROT_SPEED, true);



    // Testing the new rotate ---------------------------------------------
    double curr_yaw = DEG2RAD(355);

    double angle = DEG2RAD(-5);

    int direction = SIGN(angle);
    bool wrap = false;
    double yaw_converted;
    double start_yaw;
    double next_angle;

    next_angle = curr_yaw + angle;
    start_yaw = curr_yaw;
    if (next_angle >= DEG2RAD(360)){
        next_angle = next_angle - DEG2RAD(360);
        wrap = true;
    }
    else if (next_angle < 0){
        next_angle = next_angle + DEG2RAD(360);
        wrap = true;
    }


    for (int i = 0; i < 20; i++){
        if (wrap && curr_yaw * direction >= start_yaw * direction){
            yaw_converted = curr_yaw - direction * DEG2RAD(360);
        }
        else{
            yaw_converted = curr_yaw;
        }


        ROS_INFO("yaw: %f, converted: %f, next: %f", RAD2DEG(curr_yaw), RAD2DEG(yaw_converted), RAD2DEG(next_angle));

        // Simulating real yaw
        curr_yaw = curr_yaw + direction * DEG2RAD(1);
        if (curr_yaw >= DEG2RAD(360)){
            curr_yaw = curr_yaw - DEG2RAD(360);
        }
        else if (curr_yaw < 0){
            curr_yaw = curr_yaw + DEG2RAD(360);
        }
        if (!(yaw_converted * direction < next_angle * direction)){
            break;
        }
    }

    return 0;
}

