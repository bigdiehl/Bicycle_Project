#include "ros/ros.h"
#include "bicycle/bicycle_cmd.h"
#include <gazebo_msgs/ModelStates.h>


#include <string>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

void gazebo_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {

    std::string model_name = msg->name[0];
    
    geometry_msgs::Twist twist;
    geometry_msgs::Pose pose;

    pose = msg->pose[0];
    
    //ROS_INFO("Model name is: [%s]", model_name.c_str());
}


int main(int argc, char **argv) {

    ROS_INFO("Starting Control Test Node");

    ros::init(argc, argv, "control_test");
    
    ros::NodeHandle n;

    ros::Publisher command_pub = n.advertise<bicycle::bicycle_cmd>("bicycle_cmd", 1000);
    ros::Subscriber state_sub = n.subscribe("gazebo/model_states", 1000, gazebo_callback);

    ros::Rate loop_rate(10);

    while (ros::ok()) {

        bicycle::bicycle_cmd msg;
        msg.tau_back = 0;
        msg.tau_steering = 0;
        msg.vel_back = 0;
        msg.vel_steering = 0;

        command_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}