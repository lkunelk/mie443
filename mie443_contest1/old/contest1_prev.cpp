#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <cmath>

#include <chrono>

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180. / M_PI)
#define DEG2RAD(deg) ((deg)*M_PI / 180.)

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers = 0, desiredNLasers = 0, desiredAngle = 5;

float posX = 0.0, posY = 0.0, yaw = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Position: (%f,%f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg)
{
    //
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
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



enum State{STOP, OBS, FORW, TURN};

// Constants
float SECTOR_SIZE = 3.141 / 6; // radians
float OBS_SPEED = 1;
float TURN_SPEED = 1;
float STOP_DIST = 0.60;
State START_STATE = FORW;

// Turn Specific
float turn_time = -1;
float turn_angle;
bool new_turn = false;
State state_after_turn;

// Universal
State state = START_STATE;

void turn(float angle, State next_state=STOP){
    state = TURN;
    turn_angle = angle;
    state_after_turn = next_state;
    new_turn = true;
    
} 

void forward(){
    state = FORW;
}

void stop(){
    state = STOP;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;












    

    // Variables
    geometry_msgs::Twist twist;
 
    
    turn(3.14);
    while (ros::ok() && secondsElapsed <= 480)
    {
        ros::spinOnce();
        ROS_INFO("%f, %f", turn_time, ros::Time::now().toSec());


        // State actions ------------------------------
        if (state == OBS){
            // Rotate 360 and produce a distance histogram
            twist.linear.x = 0;
            twist.angular.z = OBS_SPEED;
            
        }
        else if (state == FORW){
            // Move forward until it reaches an obstacle
            twist.linear.x = 0.2;
            twist.angular.z = 0;
        }
        else if (state == TURN){
            // New command
            double curr_time = ros::Time::now().toSec();
            if (new_turn){
                turn_time = curr_time + abs(turn_angle) / TURN_SPEED;
                //ROS_INFO("Turning %f. Will take %f seconds", angle, turn_time - curr_time);
                new_turn = false;

                twist.linear.x = 0;
                twist.angular.z = TURN_SPEED * abs(turn_angle) / turn_angle;
            }
            // Command in action
            else if (turn_time >= curr_time){
                state = state_after_turn;
                turn_time = -1;
            }
        }
        else if (state == STOP){
            // Stop
            twist.linear.x = 0;
            twist.angular.z = 0;
        }

       




        vel_pub.publish(twist);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
        loop_rate.sleep();
    }

    return 0;
}
