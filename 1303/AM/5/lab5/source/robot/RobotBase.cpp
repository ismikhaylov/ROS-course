//
// Created by maksim on 13.11.16.
//

#include "../../heder/robot/RobotBase.h"

RobotBase::RobotBase(){
}
RobotBase::RobotBase(NodeHandle nodeHandle,string robotName,geometry_msgs::Pose currentPosition)
{
    this->nodeHandle=nodeHandle;
    this->currentPosition=currentPosition;
    this->robotName=robotName;
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot =
            nodeHandle.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;

    ifstream fin("/home/maksim/.gazebo/models/pioneer2dx/model.sdf");

    if(fin.is_open()){
        cout<<"OPEN"<<endl;
    }else cout<<"NOT OPEN"<<endl;

    string model;
    string buf;
    while(!fin.eof()){
        getline(fin, buf);
        model += buf + "\n";
    }
    srv.request.model_xml = model;
    srv.request.model_name = robotName;

    srv.request.initial_pose = currentPosition;
    add_robot.call(srv);

    publisher = nodeHandle.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
}

void RobotBase::broadcastPosition(gazebo_msgs::ModelState msg){
    cout<<robotName<<endl;
    ROS_INFO("BROADCASTER:%f %f %f",msg.pose.position.x,msg.pose.position.y,msg.pose.position.z);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) );
    tf::Quaternion q;
    q.setRPY(0, 0, msg.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robotName));
}
void RobotBase::setCurrentPosition(geometry_msgs::Pose currentPosition)
{
this->currentPosition=currentPosition;
}

