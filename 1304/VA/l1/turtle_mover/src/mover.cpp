
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <sstream>


int main(int argc, char **argv)
{
 ros::init(argc, argv, "turtleMover");
 ros::NodeHandle n;

 ros::Publisher velocPubl = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 500);
 ros::Rate loop_rate(1); //1 message per second
 int count = 0; 
 bool change=true;
 int val=3;
 while (ros::ok()) 
 {
	geometry_msgs::Twist moveMsg;
	if(change)
	{
		moveMsg.linear.x =val;
		moveMsg.linear.y =0;
		moveMsg.linear.z =0;
		moveMsg.angular.x = 0;
		moveMsg.angular.y = 0;
		moveMsg.angular.z =4;
	}else 
	{
		moveMsg.linear.x =val;
		moveMsg.linear.y =0;
		moveMsg.linear.z =0;
		moveMsg.angular.x = 0;
		moveMsg.angular.y = 0;
		moveMsg.angular.z =-4;
	}
	count++;
	if(count%5==0) { change=!change; val++;}
	if(count%100==0) val=3;
	velocPubl.publish(moveMsg);
	loop_rate.sleep();
	
}
return 0;
}
