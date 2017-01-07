#include <ros/ros.h>
#include <ctime>
#include <l2message/message_type.h>

std::string rnd_str() 
{

	std::string str = "";
	for(int i = 0; i < 10; i++)
	{
		int char_code;
		if(rand() % 100 > 50) 
			char_code = rand() % 26 + 65;
		else 
			char_code = rand() % 26 + 97;
		str += char(char_code);
	}
	return std::string(str);
}



int main(int argc, char **argv)
{
	// Initialize the ROS system.
	ros::init(argc, argv, "l2publisher");
	//publisher node 
	ros::NodeHandle pub_node;
	
	ros::Publisher publisher = pub_node.advertise<l2message::message_type>("public_data",100);
	ros::Rate loop_rate(1);

	ROS_INFO("Publisher sends messages \n");

	std::srand(time(NULL));

  while(ros::ok()) 
  {
  	//Create random message and encryption code
    std::string random_message = rnd_str();
    int random_number = rand();

    l2message::message_type pub_msg;

    pub_msg.info = random_message;
    pub_msg.codeNumber = random_number;

    //Sending a public message
    ROS_INFO("Sent message - \"%s\" \n\t\t\t\tEncryption code - %d \n", random_message.c_str(), random_number);
    publisher.publish(pub_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

	return 0;
}