#include <ros/ros.h>
#include <lab2_message/message.h>

typedef lab2_message::message message;

void callback(const message &msg) {
	message::_body_type body = msg.body;
	message::_key_type key = msg.key;
	if (key % 2 == 0) {
		ROS_INFO(body.c_str());
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "lab2_receiver");
	ROS_INFO("Start listening on node: %s", ros::this_node::getName());
	ros::NodeHandle handler;
	ros::Subscriber subscriber = handler.subscribe("secret_messages", 100, callback);
	ros::spin();
	return 0;
}
