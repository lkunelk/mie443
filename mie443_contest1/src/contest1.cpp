#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <cmath>

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>


using namespace std; 
#include <iostream>
#include <list>

#include <chrono>

#define N_BUMPER (3)
#define RAD2DEG(rad)((rad)*180./M_PI)
#define DEG2RAD(deg)((deg)*M_PI/180.)

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers = 0, desiredNLasers = 0, desiredAngle = 5;

float angular = 0.0;
float linear = 0.0;
float posX = 0.0, posY = 0.0, yaw = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position: (%f,%f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg)
{
    // 
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle) / msg->angle_increment;
    ROS_INFO("Size of laser scan array: % i and size of offset: % i ", nLasers, desiredNLasers);

    if (desiredAngle * M_PI / 180 < msg -> angle_max && -desiredAngle * M_PI / 180 > msg -> angle_min)
    {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx)
        {
            minLaserDist = std::min(minLaserDist, msg -> ranges[laser_idx]);
        }
    }
    else
    {
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx)
        {
            minLaserDist = std::min(minLaserDist, msg -> ranges[laser_idx]);
        }
    }
}



///
//
// read gmap map and transforms
// publish map and index
//


enum State{stop, obs, forw, turn};

geometry_msgs::Twist logic(){
    /*
    States:
    Rotate and observe
    Move forward
    */

    // Constants
    float SECTOR_SIZE = 3.141 / 6; // radians
    float OBS_SPEED = 1;
    float TURN_SPEED = 1;

    // Variables
    geometry_msgs::Twist twist;
    State state;
    float turn_angle;
    float curr_time, turn_time;


    // State actions ------------------------------
    if (state == obs){
        // Rotate 360 and produce a distance histogram
        twist.linear.x = 0;
        twist.angular.z = OBS_SPEED;
        
    }
    else if (state == forw){
        // Move forward until it reaches an obstacle
        twist.linear.x = 0.2;
        twist.angular.z = 0;
    }
    else if (state == turn){
        // Turn a certain angle
        twist.linear.x = 0;
        twist.angular.z = TURN_SPEED;
    }

    // State transitions ----------------------------
    if (state == obs){
        
    }
    else if (state == forw){
        if (minLaserDist < 60){

        }
    }
    else if (state == turn){
        if (turn_time > curr_time){
            state = stop;
        }

    }
    
    return twist;    
}

// The code to drive to the desired point
void drive(float xd, float yd) {
    odomCallback();
    float distance = ((posX-xd)^2+(posY-yd)^2)^0.5;
    float angle = atan((yd-posY)/(xd-posX))*180/M_PI;
    float rotation;
    if(angle - yaw > 180) {
        rotation = -(360-(angle - yaw));
    }
    else if(angle - yaw < -180) {
        rotation = -360 - (angle - yaw);
    }
    else {
        rotation = angle - yaw;
    }

    turn(rotation);
    forward(distance);

}

float roatation_to_desired_angle(int N) {
    float min_distance = std::numeric_limits<float>::infinity();
    float min_angle = std::numeric_limits<float>::infinity();
    

    float rotation;

    for(int i = 1; i < N; i++) {
        rotate(360/N);
        if(minLaserDist < min_distance) {
            min_distance = minLaserDist;
            min_angle = yaw;
        }
    }

    if(min_ngle - yaw > 180) {
        rotation = -(360-(angle - yaw));
    }
    else if(min_angle - yaw < -180) {
        rotation = -360 - (angle - yaw);
    }
    else {
        rotation = angle - yaw;
    }
    return rotation;
}


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    geometry_msgs::Point p = msg->info.origin.position;


    // int8[] data = msg->data;
    nav_msgs::MapMetaData info = msg->info;
    // uint32 height = msg->info.height;

    ROS_INFO_STREAM("map! " << p.x << ", " << p.y << ", " << p.z << "\n");
    ROS_INFO_STREAM("w/h " << info.width << ", " << info.height << ", " << "\n");

    // ros::Duration(5.0).sleep();

}

//







int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Starting!");
    ros::init(argc, argv, "nam_node");
    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("/map", 10, &mapCallback);

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


    tf::TransformListener listener;
    tf::StampedTransform robotPose;
    while(ros::ok()) {
        try {
            listener.lookupTransform("map", "base_link", ros::Time(0), robotPose);
            // ROS_INFO_STREAM("robot pose! " << robotPose.getOrigin().x() << ", " << robotPose.getOrigin().y() << "\n");
        } catch(tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        ros::spinOnce();

        ROS_INFO("Position: (%f,%f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);


        forward();
        roatate();
        stop();

        float min_distance = 1000;
        float min_angle = 1000;
        int N;

        for(int i = 1; i < N; i++) {
            rotate(360/N);
            if(minLaserDist < min_distance) {
                min_distance = minLaserDist;
                min_angle = yaw;
            }
        }
        forward();

        rotate(10);

        /*    
        //
        // Check if any of the bumpers were pressed.
        bool any_bumper_pressed = false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx)
        {
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs ::BumperEvent ::PRESSED);
        } 
        // 
        // Control logic after bumpers are being pressed.
        if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed)
        {
            angular = 0.0;
            linear = 0.2;
        }
        else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed)
        {
            angular = M_PI / 6;
            linear = 0.0;
        }
        else
        {
            angular = 0.0;
            linear = 0.0;
        }
        */

        vel = logic();
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();


    }
    ros::spin();
    return 0;
}
