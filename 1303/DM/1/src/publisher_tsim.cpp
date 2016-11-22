#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <sstream>
#include <math.h>

#define M_PIl  3.141592653589793238462643383279502884L

int main(int argc, char** argv)
{
	ros::init(argc, argv, "publisher");
	ros::NodeHandle nh;
	
	ros::Publisher Pcmd_vel = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
	ros::Rate loop_rate(1);
	
	int i=0;
	srand(time(NULL));
	while(ros::ok())
	{
		if (i>359) i=0;
		geometry_msgs::Twist msg;
		msg.linear.x = 4* cosf(i* M_PIl  /180) ;
		msg.linear.y =0 ;
		msg.linear.z =0 ;
		
		msg.angular.x=0 ;
		msg.angular.y=0 ;
		msg.angular.z= 1;
		
		ROS_INFO("linear.x= %.2f, Angular.z=%.2f", msg.linear.x, msg.angular.z);
		Pcmd_vel.publish(msg);
		ros::spinOnce();
	    loop_rate.sleep();
	    i++;	
	}
	
	return 0;
}
