#include "ros/ros.h"
#include "msg/Test.h"
void func(const msg::Test& message) {
if (message.number % 5 == 0) {
ROS_INFO("GFFG %s", message.text);
ROS_INFO("%d", message.number);
}
}
int main(int argc, char** argv) {
ros::init(argc, argv, "client_node");
ros::NodeHandle nh;
ros::Subscriber subscriber = nh.subscribe("someTopic", 1000, &func);
ros::spin();
}
