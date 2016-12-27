#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <std_msgs/String.h>
#include "robot_msg/robotMesg.h"
#include <visualization_msgs/Marker.h>

std::string target="/turtle1";
bool stopping=false;
int tm=0;

void searching(ros::NodeHandle &nh, ros::Publisher &turtle_vel)
{
	tf::TransformListener listener;
  	ros::Rate rate(10.0);
  	while (nh.ok())
  	{
    	tf::StampedTransform transform;
    	try
    	{
      		listener.lookupTransform("/turtle2", target, ros::Time(0), transform);
    	}
    	catch (tf::TransformException &e)
    	{
      		ROS_ERROR("%s", e.what());
      		ros::Duration(1.0).sleep();
      		continue;
    	}
        ROS_INFO("driving: [%s]", target.c_str());
    	geometry_msgs::Twist vel_msg;
    	vel_msg.angular.z = 6.0*atan2(transform.getOrigin().y(), transform.getOrigin().x());
    	vel_msg.linear.x = 2.0 * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
    	turtle_vel.publish(vel_msg);
    	if (vel_msg.linear.x<1.5f)
		{
            tm++;
            robot_msg::robotMesg msg;
            msg.a="/turtle2";
            ros::Publisher pub = nh.advertise<robot_msg::robotMesg>("/chatter", 100);

            ros::Rate loop_rate(2);
            for (int i = 0; i < 3; i++)//////jjjjjjjjjj
            {
                pub.publish(msg);
                loop_rate.sleep();

            }
            if(tm>=3){
                ROS_INFO("I in IF : [%s]", target.c_str());
                if (strcmp(target.c_str(), "/world")==0) break;
                target="/world";

                //sleep(2);
			}
		}
    	rate.sleep();
  	}

}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "searcher_robot");

  ros::NodeHandle node;

  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);
  ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1); ////fffffffffff
  searching(node, turtle_vel);


  return 0;
};
