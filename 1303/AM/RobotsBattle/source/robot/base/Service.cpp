//
// Created by maksim on 19.11.16.
//

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include "../../../heder/robot/base/Service.h"
#include "../../../heder/robot/base/RobotBase.h"
#include "../../../heder/robot/warior/RobotArcher.h"
Service::Service() {}

Service::Service(NodeHandle nodeHandle,string pathToModel,string modelName,geometry_msgs::Pose modelPosition) {
    this->nodeHandle=nodeHandle;
    this->modelPosition=modelPosition;
    this->pathToModel=pathToModel;
    this->modelName=modelName;

    deleteModelClient = nodeHandle.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");


    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot =nodeHandle.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;

    string model ,buf;
    ifstream fin(pathToModel);

    while(!fin.eof()){
        getline(fin, buf);
        model += buf + "\n";
    }

    fin.close();
    srv.request.model_xml = model;
    srv.request.model_name = modelName;
    srv.request.initial_pose = modelPosition;
    add_robot.call(srv);
    publisherModelToPosition = nodeHandle.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state",1000);
}

void Service::publishModelPosition(string modelName,geometry_msgs::Pose modelPosition) {
    //cout<<"PUBLISH_BASE"<<endl;
    gazebo_msgs::ModelState modelState;
    modelState.model_name =modelName;
    modelState.pose=modelPosition;
    publisherModelToPosition.publish(modelState);
}
