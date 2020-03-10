#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <visualization_msgs/Marker.h>

//define some variables here
float x, y, phi, x_goal, y_goal, x_start, y_start, phi_start;
int i = 0;

//default distance to stand away from box to take picture
float default_dist_to_box = 0.8;
float default_blank_thresh = 50000;
float default_ratio = 0.8; // As in Lowe's paper; can be tuned

#include <iostream>
#include <fstream>

#define NUM_DIGITS 3
#define OUTPUT_FILE_PATH "OUTPUT.txt"

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;

    // Params
    double dist;
    int blank_thresh;
    int num_boxes;
    double ratio;
    n.param<double>("dist_to_box", dist, default_dist_to_box);
    n.param<int>("blank_thresh", blank_thresh, default_blank_thresh);
    n.param<int>("num_boxes", num_boxes, 5);
    n.param<double>("ratio", ratio, default_ratio);

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
    
    // for(int i = 0; i < boxes.coords.size(); ++i) {
    //     std::cout << "Box coordinates: " << std::endl;
    //     std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
    //               << boxes.coords[i][2] << std::endl;
    // }


    // WHEN AT GOAL =========================================================================
    // NEEDED FROM ABOVE
    std::vector<float> goal_coord;
    bool is_done;

    // Initialize variables that store info
    int id;
    std::vector<std::string> labels;
    std::vector<std::vector<float>> coords;
    std::vector<bool> is_duplicates;
    
    // Stores which template is seen based on id
    std::vector<bool> is_seen(4, false); // 4th element (id = 3) correspond to blank

    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n, blank_thresh, ratio);

    //Initialize and store final position to go to
    x_start = robotPose.x;
    y_start = robotPose.y;
    phi_start = robotPose.phi;

    // Execute strategy.


    // create array with optimized path
    while(ros::ok()) {
        ros::spinOnce();    
        std::cout << "now: " << robotPose.x << " " << robotPose.y << " " << robotPose.phi << std::endl;
        //navigation.moveToGoal(boxes.coords[1][0] + 1.0,boxes.coords[1][1] + 1.0,boxes.coords[1][2]);
        //navigation.moveToGoal(2.0, 0.5, 0.5);

        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        if(i < num_boxes){
            x = boxes.coords[i][0];
            y = boxes.coords[i][1];
            phi = boxes.coords[i][2];
            
            navigation.moveToGoal(x + (dist*cos(phi)), y + (dist*sin(phi)), phi + 3.14);

            i = i+1;

            for (int j = 0; j<10000; j++){
                ros::spinOnce();
                id = imagePipeline.getTemplateID(boxes);
                if (id != -9){ // Not an invalid imaage
                    break;
                }
            }
            

            if (id > -1){
                // Coord
                coords.push_back(boxes.coords[i]); 

                // Label
                if (id == 3){
                    std::cout << "Recognized Blank" << std::endl;
                    labels.push_back("Blank");
                }
                else{
                    std::cout << "Recognize " << boxes.labels[id] << std::endl;
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
            //navigation.moveToGoal(x_start, y_start, phi_start);
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

    std::cout << "Writing Results" << std::endl;
    textFile.open(OUTPUT_FILE_PATH);
    textFile << "RESULTS!!!\n===============\n\n";
    for(int i = 0; i < labels.size(); i++){
        std::cout << "Writing about " << labels[i] << std::endl;
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
