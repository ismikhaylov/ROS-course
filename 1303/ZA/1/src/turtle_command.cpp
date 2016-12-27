#include "ros/ros.h" 
#include "turtlesim/Pose.h"
#include <geometry_msgs/Twist.h>
#include <stdlib.h> 
#include <time.h>
#include <math.h>

ros::Publisher send;
void getCurrentPoint(const turtlesim::Pose::ConstPtr& pose_message);

int main(int argc, char **argv)
{
  srand (time(NULL));
  ros::init(argc, argv, "turtle_command");
  ros::NodeHandle n;
  send = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  n.subscribe("/turtle1/pose", 1, getCurrentPoint);
  ros::Rate loop_rate(1);
  while(ros::ok()) {
		geometry_msgs::Twist message;
		message.linear.x = 5;
		message.linear.y = 0;
		message.linear.z = 0;
		message.angular.x = 0;
		message.angular.y = 0;
		message.angular.z = (double)(rand())/RAND_MAX*(3 - (-3)) + (-3);
		send.publish(message);	
		ros::spinOnce();
		loop_rate.sleep();
  }
  ros::spin();
  return 0;
}

void getCurrentPoint(const turtlesim::Pose::ConstPtr & pose_message) {
	std::cout << "turtle " << pose_message->x << " " << pose_message->y << " " << pose_message->theta << std::endl;
	ros::spinOnce();
}