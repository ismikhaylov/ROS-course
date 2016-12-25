//
// Created by maksim on 19.11.16.
//

#ifndef ROBOTSBATTLE_SERVICE_H
#define ROBOTSBATTLE_SERVICE_H

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
#include <mutex>
#include <gazebo_msgs/DeleteModel.h>

using namespace ros;
using std::map;
using std::cout;
using std::vector;
using std::tuple;
using std::get;
using std::mutex;
using std::string;
using std::endl;
using std::pair;
using std::to_string;
using std::ifstream;
class Service {
private:
    NodeHandle nodeHandle;
    string modelName;
    geometry_msgs::Pose modelPosition;
    string pathToModel;
    Publisher publisherModelToPosition;
public:
    ros::ServiceClient deleteModelClient ;
    gazebo_msgs::DeleteModel deleteModel;
public:
    Service();
    Service(NodeHandle nodeHandle,string pathToModel,string modelName,geometry_msgs::Pose modelPosition);
public:

    Publisher &getPublisher(){
        return publisherModelToPosition;
    }

    NodeHandle & getNodeHandle(){
        return nodeHandle;
    }

    string getModelName(){
        return modelName;
    }

    geometry_msgs::Pose &getModelPosition(){
        return modelPosition;
    }

    string getPathToModel(){
        return pathToModel;
    }


///Virtual
public:
    virtual void removeModel(){
        deleteModel.request.model_name = getModelName();
        deleteModelClient.call(deleteModel);
    }

    virtual   void setModelPosition(geometry_msgs::Pose &pose){
        modelPosition=pose;
    }
    virtual void publishModelPosition(string modelName,geometry_msgs::Pose modelPosition);
};

#endif //ROBOTSBATTLE_SERVICE_H
