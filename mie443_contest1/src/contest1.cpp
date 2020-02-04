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
        Memory() { index = 0; }
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
Position* input2DArrayOutputPositionArray(int** my2DArray, int height, int width){
    Position* array2D = 0;
    array2D = new Position[height * width];

    for (int h = 0; h < height; h++)
    {
        for (int w = 0; w < width; w++)
        {
            array2D[w + width * h].setPosition(h, w);
            // printf("%i,", w + width * h);
        }
    }

    return array2D;

}

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

Position*  minDistance(int** grid, int height, int width, int initialX, int initialY) 
{ 
    int CONFIDENCE_PERCENTAGE = 95

    // To keep track of visited QItems. Marking 
    // blocked cells as visited. 
    bool visited[height][width]; 
    // for (int i = 0; i < height; i++) { 
    //     for (int j = 0; j < width; j++) 
    //     { 
    //         // Cutoff at 95% Gmapping Occupancy confidence
    //         if (grid[i][j] < CONFIDENCE_PERCENTAGE) 
    //             visited[i][j] = true; 
    //         else
    //             visited[i][j] = false; 

    //     } 
    // } 
  
    // applying BFS on matrix cells starting from source 
    Position* path = 0;
    // Initialize array of full map size, probably inefficient
    path = new Position[height * width];

    int PATH_INDEX = 0;  
    
    QItem source(0, 0, 0); 
    std::queue<QItem> q; 
    q.push(source); 
    visited[initialX][initialY] = true; 
    while (!q.empty()) { 
        QItem p = q.front(); 
        q.pop(); 

        // add top of queue to the explored Path, wait.. this might not work
        path[PATH_INDEX] = new Position (p.row, p.col);
        PATH_INDEX++;


        // Destination found; 
        if (grid[p.row][p.col] == height * width - 1) 
            return path; 
  
        // moving up 
        if (p.row - 1 >= 0 && 
            visited[p.row - 1][p.col] == false) { 
            q.push(QItem(p.row - 1, p.col, p.dist + 1)); 
            visited[p.row - 1][p.col] = true; 
        } 
  
        // moving down 
        if (p.row + 1 < height && 
            visited[p.row + 1][p.col] == false) { 
            q.push(QItem(p.row + 1, p.col, p.dist + 1)); 
            visited[p.row + 1][p.col] = true; 
        } 
  
        // moving left 
        if (p.col - 1 >= 0 && 
            visited[p.row][p.col - 1] == false) { 
            q.push(QItem(p.row, p.col - 1, p.dist + 1)); 
            visited[p.row][p.col - 1] = true; 
        } 
  
         // moving right 
        if (p.col + 1 < width && 
            visited[p.row][p.col + 1] == false) { 
            q.push(QItem(p.row, p.col + 1, p.dist + 1)); 
            visited[p.row][p.col + 1] = true; 
        } 
    } 
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

int main(int argc, char **argv)
{
    // ros::init(argc, argv, "image_listener");
    // ros::NodeHandle nh;

    // ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    // ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

    // ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    // ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    // ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    // // Testing Memory and Position class
	// Memory mem;
    // float number = 1.1234;
	// for( int i=0; i<5; i++ )
	// { 
    //     number = number + 1.000;
	// 	mem.setPosition(number, number * 2);
	// }

	// for( int i=0; i<5; i++ )
	// { 
    //     mem.getPosition(i);
	// }

    printf("Creating a 2D array2D\n");
    printf("\n");

    int height = 5;
    int width = 10;
    int** my2DArray = create2DArray(height, width);

    // int** arrayOfCoords = DFS(my2DArray)
    
    printf("Array sized [%i,%i] created.\n\n", height, width);

    // print contents of the array2D
    printf("Array contents: \n");

    // Function to print 2D array
    output2DArray(my2DArray, height, width);

    Position* output = minDistance(my2DArray, height, width, 0, 0);

    // Position* output = input2DArrayOutputPositionArray(height, width);
    for (int w = 0; w < height * width; w++)
    {
        printf("%i, %i", output[w].getX(), output[w].getY());
        printf("\n");
    }
    printf("\n");

    // important: clean up memory
    printf("\n");
    printf("Cleaning up memory...\n");
    for ( int h = 0; h < height; h++)
    {
        delete [] my2DArray[h];
    }
    delete [] my2DArray;
    my2DArray = 0;
    printf("Ready.\n");



    // while(ros::ok() && secondsElapsed <= 60) { // TODO: increase time to 480
    //     ros::spinOnce();

    //     ROS_INFO("Position: (%f,%f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);

    //     //
    //     // Check if any of the bumpers were pressed.
    //     bool any_bumper_pressed = false;
    //     for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx)
    //     {
    //         any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs ::BumperEvent ::PRESSED);
    //     } 
    //     // 
    //     // Control logic after bumpers are being pressed.
    //     if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed)
    //     {
    //         angular = 0.0;
    //         linear = 0.2;
    //     }
    //     else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed)
    //     {
    //         angular = M_PI / 6;
    //         linear = 0.0;
    //     }
    //     else
    //     {
    //         angular = 0.0;
    //         linear = 0.0;
    //     }

    //     vel.angular.z = angular;
    //     vel.linear.x = linear;
    //     vel_pub.publish(vel);

    //     // The last thing to do is to update the timer.
    //     secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
    //     loop_rate.sleep();
    // }

    return 0;
}
