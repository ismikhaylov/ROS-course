#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Pose.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include "robot_msg/robotMesg.h"
std::string turtle_name;
std::string target="null";

int tm=0;


void chatterCallback(const robot_msg::robotMesg& msg)
{
		ROS_INFO("I heard [%s]", msg.a.c_str());
		target = msg.a;

		ros::NodeHandle node;
		ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>(turtle_name + "/cmd_vel", 1);
		tf::TransformListener listener;
		ros::Rate rate(10);
		while (node.ok())
		{
			tf::StampedTransform transform;
			try
			{
				listener.lookupTransform("/turtle1", target, ros::Time(0), transform);
			}
			catch (tf::TransformException &e)
			{
				ROS_ERROR("%s", e.what());
				ros::Duration(1.0).sleep();
				continue;
			}

			geometry_msgs::Twist vel_msg;
			vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
			vel_msg.linear.x = 2*sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
			turtle_vel.publish(vel_msg);
			if (vel_msg.linear.x<0.2f)
			tm++;
			if(tm>=200000){
				break;
			rate.sleep();
			}
		}

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "missing_robot");

  //if (argc != 2) {ROS_ERROR("need turtle name as argument"); return -1;};
   //turtle_name = argv[1];
   turtle_name = "/turtle1";
  ros::NodeHandle node;


  ros::Subscriber sub = node.subscribe("/chatter", 100, &chatterCallback); /////////////LLLLLLLLLLLLLLLL
  ros::Publisher pub = node.advertise<geometry_msgs::Twist>(turtle_name + "/cmd_vel", 1);

  geometry_msgs::Twist command;
  //initVisualization(node);
  ros::Rate loop_rate(1);
  int count = 0;
  bool change=true;
  int val=6.5;
  while(ros::ok()  && strcmp(target.c_str(), "null")==0)
  {
    geometry_msgs::Twist moveMsg;
    if(change)
    {
        moveMsg.linear.x =val;
        moveMsg.linear.y =0;
        moveMsg.linear.z =0;
        moveMsg.angular.x = 0;
        moveMsg.angular.y = 0;
        moveMsg.angular.z =-5;
    }
    else
    {
        moveMsg.linear.x =val;
        moveMsg.linear.y =0;
        moveMsg.linear.z =0;
        moveMsg.angular.x = 0;
        moveMsg.angular.y = 0;
        moveMsg.angular.z =5;
    }
    count++;
    if(count%5==0) { change=!change; val++;}
    if(count%30==0) val=3;
    pub.publish(moveMsg);
	ros::spinOnce();
	sleep(1);
	loop_rate.sleep();
  }
  return 0;
}
