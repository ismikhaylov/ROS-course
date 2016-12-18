#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "stupid_robot");

  ros::NodeHandle nh;
  ros::Publisher infoPub = nh.advertise<std_msgs::String>("stupid_robot_status",100);
  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("stupid_robot_position", 100);
  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  tf::Transform transform;
  ros::Rate loop_rate(30);
  tf::Vector3 posVector(5,5,0);

  bool isReady = false;

  sleep(1);

  while(nh.ok()){
    transform.setOrigin(posVector);
    transform.setRotation( tf::Quaternion(0, 0, 0, 1));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "place", "stupid_robot"));

    tf::StampedTransform robotTransform;
    robotTransform.setOrigin(tf::Vector3(0.0,0.0,0.0));
    robotTransform.setRotation( tf::Quaternion(0, 0, 0, 1));
    listener.waitForTransform("/place", "/clever_robot", ros::Time::now(), ros::Duration(1.0));
    try {
      ros::Time commonTime;
      std::string error;
      listener.getLatestCommonTime("/place", "/clever_robot",commonTime, &error);
      listener.lookupTransform("/place", "/clever_robot", commonTime, robotTransform);
    }
    catch (tf::TransformException &ex){
      ROS_ERROR("TransformException: %s",ex.what());
      ROS_ERROR("Trying to repeat...");
      ros::Duration(0.5).sleep();
    }

    tf::Vector3 clever = robotTransform.getOrigin();
    double dist = clever.distance(posVector);

    if(!isReady && dist <= 0.5) {
        isReady = true;
        std_msgs::String strMsg;
        strMsg.data = std::string("ready");
        infoPub.publish(strMsg);
    }

    if(isReady) {
      tf::Vector3 movement = clever - posVector;
      if(dist > 0.5 && !movement.isZero()) {
        movement.normalize();
        movement = movement * 0.08;
        posVector += movement;
      }
    } else {
      tf::Vector3 randomMovement((rand()%100-50)/50.0, (rand()%100-50)/50.0, 0.0);
      posVector += randomMovement;
    }

    visualization_msgs::Marker marker;
    geometry_msgs::Point point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.2;
    marker.pose.position = point;
    marker.header.frame_id = "stupid_robot";
    marker.header.stamp = ros::Time();
    marker.ns = "stupid_robot_namespace";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    pub.publish(marker);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
