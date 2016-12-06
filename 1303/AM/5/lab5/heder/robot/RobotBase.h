//
// Created by maksim on 13.11.16.
//

#ifndef LAB5_ROBOTBASE_H
#define LAB5_ROBOTBASE_H

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
using namespace ros;
using namespace std;
const double endX=-5,endY=-5;
class RobotBase {
public:
private:
    NodeHandle nodeHandle;
    Publisher publisher;
    string robotName;
    geometry_msgs::Pose currentPosition;
public:
    RobotBase();
    RobotBase(NodeHandle nodeHandle,string robotName,geometry_msgs::Pose currentPosition);
    void move();
    void broadcastPosition(gazebo_msgs::ModelState msg);
public:
    NodeHandle & getNodeHandle(){
        return nodeHandle;
    }
    Publisher &getPublisher(){
        return publisher;
    }
    string getRobotName(){
        return robotName;
    }
    geometry_msgs::Pose &getCurrentPosition(){
        return currentPosition;
    }

public:
    void setCurrentPosition(geometry_msgs::Pose currentPosition);
};


#endif //LAB5_ROBOTBASE_H
