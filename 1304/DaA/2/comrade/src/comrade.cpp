#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <lab2_transport/Message.h>

void message_handler(const lab2_transport::Message& message) {
	std::string m = message.message;
	int v = message.value;
    ROS_INFO("New one: %s with number: %d", m.c_str(), v);
    if (message.value % 2 == 0) {
    	ROS_INFO("Important message");
    } else {
    	ROS_INFO("Skipped message");
    }
}

int main(int argc, char **argv)
{
  std::srand(0);
  ros::init(argc, argv, "lab2_comrade");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/lab2", 100, &message_handler);
  ros::spin(); 
  return 0;
}