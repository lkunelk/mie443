#include <boxes.h>
#include <cmath.h>
#include <math.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <visualization_msgs/Marker.h>

//define variables for navigation to goal
float x, y, phi, x_goal, y_goal;
//define variables to store starting orientation
float x_start, y_start, phi_start;
//define temp variables to use to determine shortest path
float mindist, tempdist;
//define indexer for while loop and place to store index of box with min dist
int i = 0, index_of_minbox = 0;
//define distance to stand away from box to take picture
float dist = 0.8;

#include <iostream>
#include <fstream>

#define NUM_DIGITS 3
#define OUTPUT_FILE_PATH "OUTPUT.txt"

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    Navigation navigation;
    //publish to rviz marker
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    ros::Publisher vis_pub2 = n.advertise<visualization_msgs::Marker>( "visualization_marker2", 0 );
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }

    // WHEN AT GOAL =========================================================================
    // NEEDED FROM ABOVE
    std::vector<float> goal_coord;
    bool is_done;

    // Initialize variables that store info
    int id;
    std::vector<std::string> labels;
    std::vector<std::vector<float>> coords;
    std::vector<std::vector<float>> unvisitedboxes;
    std::vector<bool> is_duplicates;
    
    // Stores which template is seen based on id
    std::vector<bool> is_seen(4, false); // 4th element (id = 3) correspond to blank

    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);

    //Initialize and store final position to go to
    x_start = robotPose.x;
    y_start = robotPose.y;
    phi_start = robotPose.phi;

    // Execute strategy.
    // intialize array of unvisited boxes
    for(j=0;j<boxes.size();j++){
        std::vector<float> box;
        box.pushback(boxes.coords[j][0]);
        box.push_back(boxes.coords[j][1]);
        box.push_back(boxes.coords[j][2]);
        unvisitedboxes.push_back(box);
    }

    int RANGE = 1;
    while(ros::ok()) {
        ros::spinOnce();    
        std::cout << "now: " << robotPose.x << " " << robotPose.y << " " << robotPose.phi << std::endl;

        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        if(i < RANGE){

            for(j=0;j<unvisitedboxes.size();j++){
                //first time running put in first box coords
                if (j==0){
                    //initialize mindist and goal points to fist box
                    mindist = sqrt(pow(unvisitedboxes[j][0] - robotPose.x,2.0) + pow(unvisitedboxes[j][1] - robotPose.y,2.0));
                    x = unvisitedboxes[j][0];
                    y = unvisitedboxes[j][1];
                    phi = unvisitedboxes[j][2];
                    index_of_minbox = j;
                }

                else{
                    //find dist between next box and 
                    tempdist = sqrt(pow(unvisitedboxes[j][0] - robotPose.x,2.0) + pow(unvisitedboxes[j][1] - robotPose.y,2.0));
                    //if dist is shorter then update goal
                    if (tempdist < mindist){
                        mindist = tempdist;
                        x = unvisitedboxes[j][0];
                        y = unvisitedboxes[j][1];
                        phi = unvisitedboxes[j][2];
                        //store index of box to take out of visited box array
                        index_of_minbox = j;
                    }
                    
                }
            }
            //take out visted box from unvisited box array
            unvisitedboxes.erase(index_of_minbox);

            //move to closest box
            navigation.moveToGoal(x + (dist*cos(phi)), y + (dist*sin(phi)), phi + 3.14);

            //increase index i
            i = i+1;

            id = imagePipeline.getTemplateID(boxes);

            if (id > -1){
                // Coord

                coords.push_back(boxes.coords[i]); 

                // Label
                if (id == 3){
                    labels.push_back("Blank");
                }
                else{
                    labels.push_back(boxes.labels[id]);
                }

                // Update and Check duplicate
                if (is_seen[id]){
                    is_duplicates.push_back(true);
                }
                else{
                    is_duplicates.push_back(false);
                    is_seen[id] = true;
                }
            }

        } else {
            //send Robot back to goal
            navigation.moveToGoal(x_start, y_start, phi_start);
            break;
        }
        
        ros::Duration(0.01).sleep();
    }
    std::cout<<"Done exploring"<<std::endl;
    // WHEN FINISH EXPLORING ==========================================================================
    // Post-processing of data
    // Possibly check for double duplicates, etc.

    // Printing the data to txt file
    std::ofstream textFile;
    std::string label;
    std::string coord;
    std::string is_duplicate;

    textFile.open(OUTPUT_FILE_PATH);
    textFile << "RESULTS!!!\n===============\n\n";
    for(int i = 0; i < labels.size(); i++){
        label = labels[i];
        coord = "(" + std::to_string(coords[i][0]).substr(0, NUM_DIGITS + 1) + ", " 
        + std::to_string(coords[i][1]).substr(0, NUM_DIGITS + 1) 
        + std::to_string(coords[i][2]).substr(0, NUM_DIGITS + 1) + ")";
        if (is_duplicates[i]){
            is_duplicate = "is";
        }
        else{
            is_duplicate = "is not";
        }
        textFile << "Found " + label + " at " + coord + ". This " + is_duplicate + " a duplicate.\n";
    } 
    textFile.close();
    
    return 0;
}
