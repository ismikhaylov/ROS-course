#include <ros/ros.h>
#include <lab2_message/message.h>
#include <cstdlib>
#include <ctime>

typedef lab2_message::message message;

int main(int argc, char** argv) {
	ros::init(argc, argv, "sender");
	ros::NodeHandle handler;
	ros::Publisher publisher = handler.advertise<message>("secret_messages", 100);
	ros::Rate loop_rate(1);
	
	std::srand(std::time(0));
	
	for (int i = 0; i < 10; i++) {
		message msg;
		msg.body = "Some message...";
		msg.key = std::rand();
		publisher.publish(msg);
		ROS_INFO("Sent: %s | %d", msg.body.c_str(), msg.key);
		ros::spinOnce();
		loop_rate.sleep();
	}
}
