#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <fstream>
#include "string.h"
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <limits.h>
#include <unistd.h>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "lost_robot");
    std::cout << "Started " << std::endl;
    ros::NodeHandle node;
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot =
             node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;

    std::cout << "Reading model" << std::endl;
    ifstream fin("/home/deus/.gazebo/models/turtlebot/model.sdf");

    string model;
    string buf;

    while(!fin.eof()){
        getline(fin, buf);
        model += buf + "\n";
    }
    srv.request.model_xml = model;
    srv.request.model_name = "lost_robot";
    geometry_msgs::Pose pose;
    srv.request.initial_pose = pose;
    add_robot.call(srv);

    std::cout << "Spawn finished" << std::endl;
    ros::Publisher pub =
            node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    ros::Publisher infoPub = node.advertise<std_msgs::String>("lost_robot_status",100);
    sleep(1.0);

    std::cout << "Publisher ready" << std::endl;
    gazebo_msgs::ModelState msg;
    msg.model_name = "lost_robot";
    msg.pose.position.x = 2.0;
    msg.pose.position.y = 2.0;
    pub.publish(msg);
    std::cout << "Published" << std::endl;
    sleep(1.0);
    ros::spinOnce();

    ros::Rate rate(24.0);
    bool ready = false;
    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf::Transform transform;
    tf::Vector3 posVector(2,2,0);

    while(node.ok()) {
      transform.setOrigin( posVector );
      transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "link", "lost_robot"));

      tf::StampedTransform robot_transform;
      robot_transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
      robot_transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
      listener.waitForTransform("/link", "/helper_robot", ros::Time::now(), ros::Duration(1.0));
      try {
        ros::Time commonTime;
        std::string error;
        listener.getLatestCommonTime("/link", "/helper_robot",commonTime, &error);
        listener.lookupTransform("/link", "/helper_robot", commonTime, robot_transform);
      }
      catch (tf::TransformException &ex){
        ROS_ERROR("E: %s",ex.what());
        ros::Duration(1.0).sleep();
      }



      tf::Vector3 helper = robot_transform.getOrigin();
      double dist = helper.distance(posVector);

      if(!ready && dist <= 0.5) {
        ready = true;
        std_msgs::String strMsg;
        strMsg.data = std::string("ready");
        infoPub.publish(strMsg);
      }


      if(ready) {
        tf::Vector3 movement = helper - posVector;
        if(dist > 0.55 && !movement.isZero()) {
          movement.normalize();
          movement = movement * 0.09;
          posVector += movement;
        }
      } else {
        tf::Vector3 movement((rand()%6-3)/25.0, (rand()%6-3)/25.0, 0.0);
        posVector += movement;
      }

      gazebo_msgs::ModelState message;
      message.model_name = "lost_robot";
      double x = posVector.getX();
      double y = posVector.getY();
      double z = posVector.getZ();

      message.pose.position.x = x;
      message.pose.position.y = y;
      message.pose.position.z = z;

      pub.publish(message);
      ros::spinOnce();
      rate.sleep();

    }

    return 0;
}
