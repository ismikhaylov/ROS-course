#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "example0_b");
  ros::NodeHandle n;
  ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
  ros::Rate loop_rate(1);

  double count = 0.0;

  while (ros::ok()){
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x = count;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 2.0;
	velocity_publisher.publish(vel_msg);
	ros::spinOnce();
	loop_rate.sleep();
	count+=0.1;
}
  return 0;
}
