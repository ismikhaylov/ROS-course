#include "ros/ros.h"
#include "std_msgs/String.h"
#include "message/location.h"

void reading(const message::location::ConstPtr &coords){
	ROS_INFO("RECIEVED MESSAGE");
	ROS_INFO("coordinates recieved: < %f, %f, %f > ", coords->x, coords->y, coords->z);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reader");
	ros::NodeHandle r;

	ros::Subscriber reader = r.subscribe("coordinates", 1000, reading);
	ros::spin();
	
	return 0;
}