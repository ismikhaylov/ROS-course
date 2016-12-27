
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

bool isReady;

void onStupidRobotPositionReceived(const std_msgs::String & msg)
{
  std::string data = msg.data;
  isReady = data.compare("ready")==0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clever_robot");

  isReady = false;

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("clever_robot_position", 100);
  ros::Subscriber sub = nh.subscribe("stupid_robot_status", 100, onStupidRobotPositionReceived);

  tf::TransformBroadcaster transformBroadcastSender;
  tf::TransformListener listener;
  tf::Transform transform;

  ros::Rate loop_rate(30);

  tf::Vector3 startVector(0,0,0);
  tf::Vector3 posVector = startVector;

  sleep(1);

  while(nh.ok()){
    transform.setOrigin(posVector);
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    transformBroadcastSender.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "place", "clever_robot"));

    tf::StampedTransform robotTransform;
    robotTransform.setOrigin(tf::Vector3(0.0,0.0,0.0));
    robotTransform.setRotation(tf::Quaternion(0, 0, 0, 1));
    listener.waitForTransform("/place", "/stupid_robot", ros::Time::now(), ros::Duration(1.0));
    try {
      ros::Time commonTime;
      std::string error;
      listener.getLatestCommonTime("/place", "/stupid_robot",commonTime, &error);
      listener.lookupTransform("/place", "/stupid_robot", commonTime, robotTransform);
    }
    catch (tf::TransformException &ex){
      ROS_ERROR("TransformException: %s",ex.what());
      ROS_ERROR("Trying to repeat...");
      ros::Duration(0.5).sleep();
    }

    tf::Vector3 target = robotTransform.getOrigin();

    double dist = target.distance(posVector);

    if(!isReady) {
      if(dist >= 0.3) {
        tf::Vector3 movement = target - posVector;
        if(!movement.isZero()) {
          movement.normalize();
          movement = movement * 0.08;
          posVector += movement;
        }
      } else {
        ros::Duration(0.5).sleep();
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
    point.z = 0.2;
    marker.pose.position = marker.pose.position = point;
    marker.header.frame_id = "clever_robot";
    marker.header.stamp = ros::Time();
    marker.ns = "clever_robot_namespace";
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
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    pub.publish(marker);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
