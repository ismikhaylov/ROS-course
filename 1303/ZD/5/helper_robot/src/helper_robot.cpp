#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <std_msgs/String.h>
#include <fstream>
#include "string.h"
#include <iostream>

using namespace std;


bool status;

void listenerCallback(const std_msgs::String & msg)
{
  std::string data = msg.data;
  if(data.compare("ready")==0) {
    status = true;
    ROS_INFO("Robot ready for return!");
  }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "helper_robot");
    std::cout << "Start" << std::endl;
    ros::NodeHandle node;
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot =
             node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;

    ros::Subscriber sub = node.subscribe("lost_robot_status", 100, listenerCallback);

    std::cout << "Reading model" << std::endl;
    ifstream fin("/home/deus/.gazebo/models/turtlebot/model.sdf");

    string model;
    string buf;

    while(!fin.eof()){
        getline(fin, buf);
        model += buf + "\n";
    }
    srv.request.model_xml = model;
    srv.request.model_name = "helper_robot";
    geometry_msgs::Pose pose;
    srv.request.initial_pose = pose;
    add_robot.call(srv);

    std::cout << "Spawn finished" << std::endl;
    ros::Publisher pub =
            node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    sleep(1.0);
    std::cout << "Publisher ready" << std::endl;
    gazebo_msgs::ModelState msg;
    msg.model_name = "helper_robot";
    msg.pose.position.x = -3.0;
    msg.pose.position.y = -3.0;
    pub.publish(msg);
    std::cout << "Published" << std::endl;
    sleep(1.0);
    ros::spinOnce();
    ros::spinOnce();

    ros::Rate rate(24.0);
    bool ready = false;
    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf::Transform transform;
    tf::Vector3 startVector(-3,-3,0);
    tf::Vector3 posVector = startVector;

    while(node.ok()) {
      transform.setOrigin( posVector );
      transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "link", "helper_robot"));


      tf::StampedTransform robot_transform;
      robot_transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
      robot_transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
      listener.waitForTransform("/link", "/lost_robot", ros::Time::now(), ros::Duration(1.0));
      try {
        ros::Time commonTime;
        std::string error;
        listener.getLatestCommonTime("/link", "/lost_robot",commonTime, &error);
        listener.lookupTransform("/link", "/lost_robot", commonTime, robot_transform);
      }
      catch (tf::TransformException &ex){
        ROS_ERROR("E: %s",ex.what());
        ros::Duration(1.0).sleep();
      }

      tf::Vector3 target = robot_transform.getOrigin();

      double dist = target.distance(posVector);

      if(!status) {
        if(dist >= 0.6) {
          tf::Vector3 movement = target - posVector;
          if(!movement.isZero()) {
            movement.normalize();
            movement = movement * 0.08;
            posVector += movement;
          }
        } else {
          ros::Duration(1.0).sleep();
        }
      } else {
        tf::Vector3 movement = startVector - posVector;
        double startDist = startVector.distance(posVector);
        if(startDist > 0.3 && !movement.isZero()) {
          movement.normalize();
          movement = movement * 0.08;
          posVector += movement;
        }
      }


      gazebo_msgs::ModelState message;
      message.model_name = "helper_robot";
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
