#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <visualization_msgs/Marker.h>
#include "robot_msg/robotMesg.h"
std::string turtle_name;

ros::Publisher marker_pub;
visualization_msgs::Marker robotM;

void initVisualization(ros::NodeHandle &nh)
{
 marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
 robotM.header.frame_id = "turtle2";
 robotM.header.stamp = ros::Time::now();
 robotM.ns ="turtle2_ns";
 robotM.action = visualization_msgs::Marker::ADD;
if(strcmp(turtle_name.c_str(), "/turtle1")==0)
  {
    robotM.color.r = 1.0f;
    robotM.id=1;
  }
  else {
   robotM.color.b=1.0f;
   robotM.id=2;
  }

 robotM.type = visualization_msgs::Marker::SPHERE;
 robotM.pose.orientation.x = 0.0;
 robotM.pose.orientation.y = 0.0;
 robotM.pose.orientation.z = 0.0;
 robotM.pose.orientation.w = 0.0;


 robotM.color.a = 0.8;
 robotM.scale.x = 0.8;
 robotM.scale.y = 0.8;
 robotM.scale.z = 0.8;

}

void poseCallback(const turtlesim::PoseConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transform.setRotation(q);
  //ROS_INFO("eeeeeeeeeeeeeeeeeeeeeeeee ]dddddddddddd");
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
  robotM.pose.position.x = msg->x;
	robotM.pose.position.y = msg->y;
	robotM.pose.position.z = 0;
	marker_pub.publish(robotM);

}



int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
  turtle_name = argv[1];

  ros::NodeHandle node;
  initVisualization(node);
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);
  ros::spin();
  return 0;
};
