#include "ros/ros.h"
#include "std_msgs/String.h"
#include <roc_message/Messager.h>
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sendRocket");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<roc_message::Messager>("chatter", 1000);
  ros::Rate loop_rate(0.2);

  int count = 0;

  while (ros::ok()){
	roc_message::Messager msg;
	srand(time(NULL));
	int forRand = 1 + rand() % 3;
 	ROS_INFO("%d", forRand);
	std::stringstream ss;
	ss<<forRand;
   	msg.data = ss.str();
	
   	ROS_INFO("Rocket sent to the zone [%s]", msg.data.c_str());
	chatter_pub.publish(msg);

    	ros::spinOnce();
	
    	loop_rate.sleep();
    	++count;
  }
  return 0;
}
