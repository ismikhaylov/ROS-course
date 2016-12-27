#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include "cleaner.h"

int command = 0;

void callCommand(const std_msgs::Int16::ConstPtr& msg) 
{
    command = msg->data;
}

int main(int argc, char** argv) 
{
    gazebo::client::setup(argc, argv);
    ros::init(argc, argv, "alg");
    ros::NodeHandle nodeHandle;
    ros::Subscriber sub = nodeHandle.subscribe("scout", 1000, callCommand);

    init( "cleaner" );  


    ros::Rate rate(10);


    while (ros::ok() && connand != 3)
    {
        if (command == 3) {
            removeModel("cleaner");
        } else if (command != 1) {            
            execute();
        } else {
            ROS_INFO("Robot paused!");
        }

        gazebo::common::Time::MSleep(100);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("END cleaner");
    gazebo::shutdown();
    return 0;
}