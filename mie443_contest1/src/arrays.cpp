// C++ program to find the shortest path between 
// a given source cell to a destination cell. 
#include <bits/stdc++.h> 
#include <ros/console.h>
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include<nav_msgs/Odometry.h>
#include<tf/transform_datatypes.h>

#include <stdio.h>
#include <cmath>
using namespace std; 

//To store matrix cell cordinates 
struct Point 
{ 
	int x; 
	int y; 
}; 

// A Data Structure for queue used in BFS 
struct queueNode 
{ 
	Point pt; // The cordinates of a cell 
	int dist; // cell's distance of from the source 
}; 

// Check whether given cell (row, col) is a valid 
// cell or not (in range) 
bool isValid(int row, int col, int ROW, int COL) 
{ 
	return (row >= 0) && (row < ROW) && 
		(col >= 0) && (col < COL); 
} 



void printpath(vector<queueNode>& path) 
{ 
    cout << "Path: ";
    int size = path.size(); 
    for (int i = 0; i < size; i++)  
        cout << path[i].pt.x << "," << path[i].pt.y << " ";     
    cout << endl; 
} 

// Helper function to convert 2D array of width (numCol) to 1D row-major array coordinates
int convert2Dto1D(int row, int col, int numCol)
{
    return col + numCol * row;
}

// Check a NxN grid around a point
bool checkNeighboursWithDepth(int mat[], int range, int x, int y, int numRow, int numCol)
{ 
    for(int i = -range; i <= range; i++){
        for(int j = -range; j <= range; j++){
            int row = x + i; 
			int col = y + j;
            cout << row << ", " << col << " :" << mat[convert2Dto1D(row, col, numCol)];
            cout << endl;
            
            if (!isValid(row, col, numRow, numCol) || mat[convert2Dto1D(row, col, numCol)] != -1 )
			{ 
                return false;
            }

        }
    }

	// Return true if destination cannot be reached 
	return true; 
}

// function to find the shortest path between 
// a given source cell to a destination cell. 
vector<queueNode>  BFS(int mat[], int numRow, int numCol, Point src) 
{ 
	int MIN_PATH_SIZE = 5;
	int MAX_OCCUPIED_PROB = 50;

	// These arrays are used to get row and column 
	// numbers of 4 neighbours of a given cell 
	int rowNum[] = {-1, 0, 0, 1}; 
	int colNum[] = {0, -1, 1, 0}; 

    vector<queueNode> path; // Store path history
	queue<vector<queueNode> > q;  // BFS queue
	bool visited[convert2Dto1D(numRow, numCol, numCol) - 1]; 
	memset(visited, false, sizeof visited); 
	
	// Mark the source cell as visited 
	visited[convert2Dto1D(src.x, src.y, numCol)] = true; 

	// Distance of source cell is 0 
	queueNode s = {src, 0}; 
    path.push_back(s); 	
	q.push(path); // Enqueue source cell 

	while (!q.empty()) 
	{ 
        path = q.front();

		queueNode curr = path[path.size() - 1]; 
		Point pt = curr.pt; 

		// If we have reached the destination cell, 
		// we are done 
		// if (mat[pt.x][pt.y] == 20 || pt.x == dest.x && pt.y == dest.y) 
		
        // cout << convert2Dto1D(pt.x, pt.y, numCol) << " ";
        if (mat[convert2Dto1D(pt.x, pt.y, numCol)] == -1 && path.size() >= MIN_PATH_SIZE) 
        {
            if (checkNeighboursWithDepth(mat, 2, pt.x, pt.y, numRow, numCol)){
                return path; 
            }
        }

        q.pop();

		for (int i = 0; i < 4; i++) 
		{ 
			int row = pt.x + rowNum[i]; 
			int col = pt.y + colNum[i]; 

			if (isValid(row, col, numRow, numCol) 
				&& mat[convert2Dto1D(row, col, numCol)] < MAX_OCCUPIED_PROB 
				&& !visited[convert2Dto1D(row, col, numCol)])
			{ 
				// mark cell as visited and enqueue it 
				visited[convert2Dto1D(row, col, numCol)] = true; 
				queueNode Adjcell = { {row, col}, 
									curr.dist + 1 }; 

                vector<queueNode> newpath(path);
                newpath.push_back(Adjcell); 

				q.push(newpath); 
			} 
		} 
	} 

	// Return blank vector if destination cannot be reached 
    vector<queueNode> noPath; 
	return noPath; 
} 

int main(int argc, char **argv) 
{ 
    ROS_INFO_STREAM("Starting!");
    ros::init(argc, argv, "yuxiang");
    ros::NodeHandle n;
    ros::Publisher coord_pub = n.advertise<geometry_msgs::Point>("coord", 10);

	ros::Rate loop_rate(10);
    int ROW = 8;
    int COL = 7;
	int mat[convert2Dto1D(ROW, COL, COL) - 1] = 
	{  
		  0,  -1, -1,    0, 0, 1, -1,
          -1, -1,  0,   -1,  0,  1, -1,
          0,  -1,  -1,  -1, -1, 0, -1,
          1,   -1, -1,  -1, -1, -1, -1,
          1,   1,  -1,  -1, -1, -1, -1,
          1,   1,  -1,  -1, -1, -1, -1,
          1,   1,  -1,  -1, -1, -1, -1,
          1,   1,  -1,  -1, -1, -1, -1
	}; 

	Point source = {0, 0}; 

	vector<queueNode> path = BFS(mat, ROW, COL, source); 

    printpath(path);

	if (path.size() >= 1)
	{
		queueNode curr = path[path.size() - 1];
		geometry_msgs::Point msg;

		msg.x = curr.pt.x;
		msg.y = curr.pt.y;

		while (ros::ok())
		{
			coord_pub.publish(msg);
			ros::spinOnce();
		}
	}
	else
	{
		ROS_INFO("No path found");
	}

	return 0;
} 
