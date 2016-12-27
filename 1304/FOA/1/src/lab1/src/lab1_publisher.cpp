#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "publisher");
	ros::NodeHandle n;

	ros::Publisher direction_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	ros::Rate loop_rate(1);

	int count = 0;
	srand(time(NULL));

	while(ros::ok()){
		geometry_msgs::Twist vel_msg;
		vel_msg.linear.x = (double)(rand() % 10 + 1) / 4.0;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = (double)(rand() % 10 - 5) / 2.0;

		ROS_INFO("{RANDOM WALK} linear.x = %.2f, angular.z = %.2f\n", vel_msg.linear.x, vel_msg.angular.z);

		direction_publisher.publish(vel_msg);
		ros::spinOnce();

		loop_rate.sleep();
		count++;
	}
	return 0;
}