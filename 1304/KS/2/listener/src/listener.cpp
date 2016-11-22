#include <ros/ros.h>
#include "mess/Information.h"

void scoutCallback(const mess::Information::ConstPtr& msg) {
	if (msg->number % 2) {
 		ROS_INFO("Message: %s", msg->data.c_str());
	} else {		
 		ROS_WARN("Message: %s", msg->data.c_str());
	}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lst_confederate");
  ros::NodeHandle nodeHandle;
  ros::Subscriber sub = nodeHandle.subscribe("scout", 1000, scoutCallback);
  ros::spin();
  return 0;
}