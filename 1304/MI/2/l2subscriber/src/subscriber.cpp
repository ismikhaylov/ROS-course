#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <l2message/message_type.h>

//handler function, processing received messages
void msg_handler(const l2message::message_type& rec_msg)
{
	int dectyptionNumber = rec_msg.codeNumber;
	std::string msg_text = rec_msg.info;
//Print recived message and code
	ROS_INFO("Recived message - \"%s\" \n\t\t\t\tDecryption code - %d", msg_text.c_str(), dectyptionNumber);
//Decrypt code
	if(dectyptionNumber % 2 == 0)
		ROS_INFO("The result of the decryption - meaningful message \n");
	else 
		ROS_INFO("The result of the decryption - not meaningful message \n");
}

int main(int argc, char **argv)
{
	// Initialize the ROS system.
	ros::init(argc, argv, "l2subscriber");
	//Subscriber node
	ros::NodeHandle sub_node;

	ros::Subscriber subscriber = sub_node.subscribe("public_data", 100, &msg_handler);
	ros::spin(); 
	
	return 0;
}