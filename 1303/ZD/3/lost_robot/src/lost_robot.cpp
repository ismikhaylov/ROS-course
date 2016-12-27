#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "lost_robot");
  ROS_INFO("Lost robot starting...");
  ros::NodeHandle nh;
  ros::Publisher infoPub = nh.advertise<std_msgs::String>("lost_robot_status",100);
  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("lost_robot_position", 100);
  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  tf::Transform transform;
  ros::Rate loop_rate(30);
  tf::Vector3 posVector(11,13,0);

  double _colorR = 0.10;
  double _colorG = 0.98;
  double _colorB = 0.92;

  bool ready = false;

  sleep(1);

  while(nh.ok()){
    transform.setOrigin( posVector );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "lost_robot"));

    tf::StampedTransform robot_transform;
    robot_transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
    robot_transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    listener.waitForTransform("/world", "/helper_robot", ros::Time::now(), ros::Duration(1.0));
    try {
      ros::Time commonTime;
      std::string error;
      listener.getLatestCommonTime("/world", "/helper_robot",commonTime, &error);
      listener.lookupTransform("/world", "/helper_robot", commonTime, robot_transform);
    }
    catch (tf::TransformException &ex){
      ROS_ERROR("E: %s",ex.what());
      ros::Duration(1.0).sleep();
    }

    tf::Vector3 helper = robot_transform.getOrigin();
    double dist = helper.distance(posVector);

    if(!ready && dist <= 0.2) {
      if(_colorB >= 0.1) {
        _colorB -= 0.05;
      } else {
        ready = true;
        std_msgs::String strMsg;
        strMsg.data = std::string("ready");
        infoPub.publish(strMsg);
      }
    }

    if(ready) {
      tf::Vector3 movement = helper - posVector;
      if(dist > 1.3 && !movement.isZero()) {
        movement.normalize();
        movement = movement * 0.09;
        posVector += movement;
      }
    } else {
      tf::Vector3 movement((rand()%6-3)/25.0, (rand()%6-3)/25.0, 0.0);
      posVector += movement;
    }

    visualization_msgs::Marker marker;
    geometry_msgs::Point point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.16;
    marker.pose.position = marker.pose.position = point;
    marker.header.frame_id = "lost_robot";
    marker.header.stamp = ros::Time();
    marker.ns = "lost_robot_namespace";
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
    marker.color.r = _colorR;
    marker.color.g = _colorG;
    marker.color.b = _colorB;
    pub.publish(marker);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
