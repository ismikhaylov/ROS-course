//
// Created by maksim on 12.11.16.
//

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <tf/transform_listener.h>

#include "../heder/robot/RobotBase.h"
#include "../heder/robot/RobotLost.h"

using namespace std;


int main(int argc, char **argv) {
    ros::init(argc,argv,"node1");
    ros::NodeHandle robotNodeLost;
    string robotNameLost="r1";
    geometry_msgs::Pose pose;
    pose.position.x=5;
    pose.position.y=5;
    RobotLost robotLost(robotNodeLost,robotNameLost,pose);
    robotLost.sendHelpMassage();

    robotLost.m1();

    return 0;
}
