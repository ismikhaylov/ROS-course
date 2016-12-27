#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

bool status;

void listenerCallback(const std_msgs::String & msg)
{
  std::string data = msg.data;
  if(data.compare("ready")==0) {
    status = true;
    ROS_INFO("Robot ready for return!");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "helper_robot");
  status = false;
  ROS_INFO("Lost robot starting...");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("helper_robot_position", 100);
  ros::Subscriber sub = nh.subscribe("lost_robot_status", 100, listenerCallback);
  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  tf::Transform transform;
  ros::Rate loop_rate(30);

  tf::Vector3 startVector(14,-13,0);
  tf::Vector3 posVector = startVector;

  sleep(1);

  while(nh.ok()){
    transform.setOrigin( posVector );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "helper_robot"));

    tf::StampedTransform robot_transform;
    robot_transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
    robot_transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    listener.waitForTransform("/world", "/lost_robot", ros::Time::now(), ros::Duration(1.0));
    try {
      ros::Time commonTime;
      std::string error;
      listener.getLatestCommonTime("/world", "/lost_robot",commonTime, &error);
      listener.lookupTransform("/world", "/lost_robot", commonTime, robot_transform);
    }
    catch (tf::TransformException &ex){
      ROS_ERROR("E: %s",ex.what());
      ros::Duration(1.0).sleep();
    }

    tf::Vector3 target = robot_transform.getOrigin();

    double dist = target.distance(posVector);

    if(!status) {
      if(dist >= 0.15) {
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

    visualization_msgs::Marker marker;
    geometry_msgs::Point point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.16;
    marker.pose.position = marker.pose.position = point;
    marker.header.frame_id = "helper_robot";
    marker.header.stamp = ros::Time();
    marker.ns = "helper_robot_namespace";
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
    marker.color.g = 0.2;
    marker.color.b = 0.2;
    pub.publish(marker);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
