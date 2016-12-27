#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char** argv) {

	ros::init(argc, argv, "lab1");
	ros::NodeHandle node;

	ros::Publisher publisher = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
	ros::Rate loop_rate(1);

	geometry_msgs::Twist msg;
	msg.linear.x += 20;
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = 0;
	
	publisher.publish(msg);
	ros::spinOnce();
	loop_rate.sleep();

	while(ros::ok()) {
		geometry_msgs::Twist msg;
		msg.linear.x += 1;
		msg.linear.y = 0;
		msg.linear.z = 0;
		msg.angular.x = 0;
		msg.angular.y = 0;
		msg.angular.z += 2;
		
		publisher.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
