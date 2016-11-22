#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main (int argc, char **argv) {
    ros::init(argc, argv, "turtle_random_move");
    ros::NodeHandle nodeHandle;
	ros::Rate loop(1);
	ros::Publisher pub = nodeHandle.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);	

	while (ros::ok) {
		geometry_msgs::Twist msg;
		msg.linear.x = (double) (rand() % 3);
		msg.linear.y = 0;
		msg.linear.z = 0;

		msg.angular.x = 0;
		msg.angular.y = 0;
		msg.angular.z = (double) (rand() % 10) - 5.0;
	
		ROS_INFO("Random move linear x = %.2f, angular z = %.2f", msg.linear.x, msg.angular.z);
		pub.publish(msg);

		ros::spinOnce();
		loop.sleep();
	}

	return 0;
}