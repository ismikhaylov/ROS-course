#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"


int main (int argc, char **argv) {
        ros::init(argc, argv, "hello_world");
        ros::NodeHandle nh;
        ROS_INFO_STREAM("Hello, ROS!");
	ros::Rate loop(1);

	ros::Publisher velo_p = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	while (ros::ok) {
		geometry_msgs::Twist msg;

		msg.linear.x = (double) (rand() % 10 + 1) / 4.0;
		msg.linear.y = 0;
		msg.linear.z = 0;

		msg.angular.x = 0;
		msg.angular.y = 0;
		msg.angular.z = (double) (rand() % 10 -5) / 2.0;
	
		ROS_INFO("Random move x = %.2f, z = %.2f\n", msg.linear.x, msg.angular.z);
		velo_p.publish(msg);

		ros::spinOnce();
		loop.sleep();
	}

	return 0;
}

