//
// Created by maksim on 12.11.16.
//

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_listener.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <tf/transform_listener.h>


#include "math.h"
#include "../heder/robot/RobotBase.h"
#include "../heder/robot/RobotHelper.h"

using namespace std;


int main(int argc, char **argv) {
    ros::init(argc,argv,"node2");
    ros::NodeHandle robotNodeHelper;
    string robotNameHelper="r2";
    geometry_msgs::Pose pose;
    //pose.position.x=2;
    //pose.position.y=2;
    pose.position.x=-5;
    pose.position.y=-5;

    RobotHelper robotHelper(robotNodeHelper,robotNameHelper,pose);

   // robotHelper.m2();
    cout<<"main1"<<endl;
    ros::spin();
    cout<<"main2"<<endl;

    return 0;
}

