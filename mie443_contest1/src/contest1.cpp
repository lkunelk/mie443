#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_datatypes.h>
#include <stdio.h>
#include <cmath>

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

class Position 
{
    uint64_t secondsElapsed;
    int x, y;
	public:
        Position(int posX, int posY) { 
            x = posX;
            y = posY;
        }

        void setPosition(int posX, int posY)
        {
            x = posX;
            y = posY;
        }
        int getX() { return x; }
        int getY() { return y; }

};

class Memory
{   
    Position poseArray[100]; // TODO: make sure this is big enough
    private:
        int index;
	public:
        // Memory() { index = 0; }
        void setPosition(float posX, float posY)
        {
            poseArray[index].setPosition(posX, posY);
            ROS_INFO("Set Position at index %i: (%f, %f).", index, posX, posY);
            index++;
        } 
        void getPosition(int index)
        {
            float x = poseArray[index].getX(), y = poseArray[index].getY();
            ROS_INFO("Position at index %i: (%f, %f).", index, x, y);
        }
};

// Returns a pointer to a newly created 2d array the array2D has size [height x width]

int** create2DArray(unsigned height, unsigned width)
{
    int** array2D = 0;
    array2D = new int*[height];

    for (int h = 0; h < height; h++)
    {
        array2D[h] = new int[width];

        for (int w = 0; w < width; w++)
        {
                // fill in some initial values
                // (filling in zeros would be more logic, but this is just for the example)
                array2D[h][w] = w + width * h;
        }
    }

    return array2D;
}


void output2DArray(int** my2DArray, int height, int width) 
{ 
    for (int h = 0; h < height; h++)
    {
        for (int w = 0; w < width; w++)
        {
                printf("%i,", my2DArray[h][w]);
        }
        printf("\n");
    }
}

// Template function to output Position array
// Position* input2DArrayOutputPositionArray(int** my2DArray, int height, int width){
//     Position* array2D = 0;
//     array2D = new Position[height * width];

//     for (int h = 0; h < height; h++)
//     {
//         for (int w = 0; w < width; w++)
//         {
//             array2D[w + width * h].setPosition(h, w);
//             // printf("%i,", w + width * h);
//         }
//     }

//     return array2D;

// }

class QItem { 
    public: 
        int row; 
        int col; 
        int dist; 
        QItem(int x, int y, int w) 
            : row(x), col(y), dist(w) 
        { 
        } 
};

//graph class for DFS travesal
class DFSGraph
{
    int V;                                // No. of vertices
    std::list<int> *adjList;
    std::list<int> path;    
    void DFS_util(int v, bool visited[]); // A function used by DFS
    public:
        // class Constructor
        DFSGraph(int V)
        {
            this->V = V;
            // this->currentDepth = 0;
            adjList = new list<int>[V];
            // path = new list<int>;
        }
        // function to add an edge to graph
        void addEdge(int v, int w)
        {
            adjList[v].push_back(w); // Add w to v’s list.
        }

        list<int> DFS(); // DFS traversal function
};
void DFSGraph::DFS_util(int v, bool visited[])
{
    // current node v is visited
    visited[v] = true;

    path.push_back(v);
    cout << v << " ";

    // recursively process all the adjacent vertices of the node
    list<int>::iterator i;
    for (i = adjList[v].begin(); i != adjList[v].end(); ++i)
        if (!visited[*i])
            DFS_util(*i, visited);
}

// DFS traversal
list<int> DFSGraph::DFS()
{
    // initially none of the vertices are visited
    bool *visited = new bool[V];
    for (int i = 0; i < V; i++)
        visited[i] = false;

    // explore the vertices one by one by recursively calling  DFS_util
    for (int i = 0; i < V; i++)
        if (visited[i] == false)
            DFS_util(i, visited);

    return path;
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

#include <ros/console.h>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>

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
    }
    ros::spin();
    return 0;
}
