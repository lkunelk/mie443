#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <visualization_msgs/Marker.h>

//define some variables here
float x, y, phi, x_goal, y_goal;
int i = 0;
float dist = 1.2;


int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    Navigation navigation;
    //publish to rviz marker
    ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
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
        // if(i < boxes.coords.size()){
        //     x = boxes.coords[i][0];
        //     y = boxes.coords[i][1];
        //     phi = boxes.coords[i][2];
            
        //     navigation.moveToGoal(x + (dist*cos(phi)), y + (dist*sin(phi)), phi);

        //     i = i+1;

        //     imagePipeline.getTemplateID(boxes);

        // }

        x = boxes.coords[0][0];
        y = boxes.coords[0][1];
        phi = boxes.coords[0][2];
        x_goal = x + (dist*cos(phi));
        y_goal = y + (dist*sin(phi));
        std::cout << "goal: " << x_goal << " " << y_goal << " " << phi << std::endl;
        
        //send box to rviz
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        vis_pub.publish( marker );

        //send goal to rviz
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x_goal;
        marker.pose.position.y = y_goal;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        vis_pub.publish( marker );

        navigation.moveToGoal(x_goal, y_goal, phi);
        imagePipeline.getTemplateID(boxes);
        
        ros::Duration(0.01).sleep();
    }
    return 0;
}
