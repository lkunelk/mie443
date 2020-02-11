#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <math.h>

#define RAD2DEG(rad) ((rad)*180. / M_PI)
#define SIGN(num) (std::abs(num) / num)

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg)
{
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}

class Move{
    private:
        ros::Publisher vel_pub;
        ros::Subscriber bumper_sub;
        geometry_msgs::Twist vel;

        double next_update; // Time (s) when the current movement will be stopped
        bool bumped; //Whether or not a collision has occured

    public:
    Move(ros::NodeHandle nh){
        vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
        bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
        reset_bumped();
        ROS_INFO("Mover initiated.");
    }

    void forward(double distance, double speed, bool verbose=false){
        bool going_backwards = SIGN(speed) == -1 || SIGN(distance) == -1;

        next_update = ros::Time::now().toSec() + std::abs(distance) / std::abs(speed);

        vel.linear.x = speed * SIGN(distance);
        vel.angular.z = 0;

        if (verbose){
            ROS_INFO("Moving %f m, will take %f s, will finish at %f s", distance, std::abs(distance) / std::abs(speed), next_update);
        }

        while(ros::ok() && ros::Time::now().toSec() < next_update){
            if (is_collision() && !going_backwards){
                // Regarding 2nd term: Can still go backwards when bumper pressed
                ROS_WARN("Collision!");
                stop(true);
                bumped = true;
                break;
            }
            vel_pub.publish(vel);
        }
    }

    void rotate(double angle, double speed, bool verbose=false){
        int direction = SIGN(angle)
        bool wrap = false;
        float yaw_converted;
        float start_yaw;
        // Angle is in radians, speed is in radians per second
        if (std::abs(angle) > 2 * M_PI){
            ROS_ERROR("Angle given might be in degrees. ALSO DOESN'T SUPPORT MORE THAN 360 degrees");
        }

        //next_update = ros::Time::now().toSec() + std::abs(angle) / std::abs(speed);
        next_angle = curr_yaw + angle;
        start_yaw = curr_yaw;
        if next_angle > DEG2RAD(360){
            next_angle = next_angle - DEG2RAD(360);
            wrap = true;
        }
        else if next_angle < 0{
            next_angle = next_angle + DEG2RAD(360);
            wrap = true
        }

        vel.linear.x = 0;
        vel.angular.z = speed * SIGN(angle);

        if (verbose){
            ROS_INFO("Rotating %f deg, will take %f s, will finish at %f s", RAD2DEG(angle), std::abs(angle) / std::abs(speed), next_update);    
        }

        //while(ros::ok() && ros::Time::now().toSec() < next_update){
        ros::spinOnce();
        yaw_converted = curr_yaw
        while(ros::ok() && direction * yaw_converted < direction * next_angle){
            if (is_collision()){
                ROS_WARN("Collision!");
                stop(true);
                bumped = true;
                break;
            }
            vel_pub.publish(vel);
            ros::spinOnce();
            if (wrap && direction * start_yaw < direction * curr_yaw < direction * DEG2RAD(360)){
                yaw_converted = curr_yaw - 360;
            }
            else{
                yaw_converted = curr_yaw
            }
        }
    }

    void stop(bool verbose=false){
        if (verbose){
            ROS_INFO("STOPPED");
        }

        vel.linear.x = 0;
        vel.angular.z = 0;
        vel_pub.publish(vel);
    }

    double get_next_update(){
        return next_update;
    }

    double is_bumped(){
        return bumped;
    }
    void reset_bumped(){
        bumped = false;
    }

    bool is_collision(){
        ros::spinOnce(); // Update the bumper states
        return bumper[kobuki_msgs::BumperEvent::LEFT] == kobuki_msgs::BumperEvent::PRESSED ||
            bumper[kobuki_msgs::BumperEvent::CENTER] == kobuki_msgs::BumperEvent::PRESSED ||
            bumper[kobuki_msgs::BumperEvent::RIGHT] == kobuki_msgs::BumperEvent::PRESSED;
    }

};

