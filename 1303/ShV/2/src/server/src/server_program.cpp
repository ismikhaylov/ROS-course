#include "ros/ros.h"
#include "msg/Test.h"
int main(int argc, char** argv) {
ros::init(argc, argv, "server_node");
ros::NodeHandle nh;
ros::Rate loop_rate(1); 
ros::Publisher publisher = nh.advertise<msg::Test>("someTopic",1000);
msg::Test message;
int i = 0; 
while(i <= 100) {
message.number = ++i;
message.text = "Some Text";
publisher.publish(message);
loop_rate.sleep();
}
}
