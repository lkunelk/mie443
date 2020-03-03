#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
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
    ImagePipeline imagePipeline(n);


    // Execute strategy.
    while(ros::ok()) {
        ros::spinOnce();
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi????????????????????
        id = imagePipeline.getTemplateID(boxes);
        if (id > -1){
            // Coord
            coords.push_back(goal_coord); 

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
        ros::Duration(0.01).sleep();
    }

    // WHEN FINISH EXPLORING ==========================================================================
    if (is_done){
        // Post-processing of data
        // Possibly check for double duplicates, etc.

        // Printing the data to txt file
        std::ofstream textFile;
        std::string label;
        std::string coord;
        std::string is_duplicate;

        textFile.open(OUTPUT_FILE_PATH);
        for(int i = 0; i < 5; ++i){
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
    }
    return 0;
}
