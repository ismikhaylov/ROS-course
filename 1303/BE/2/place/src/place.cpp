#include "ros/ros.h"
#include "place/Phrase.h"

#include <iostream>
#include <fstream>
#include <string>
using namespace std;

void chatterCallback(const place::Phrase::ConstPtr& msg)
{
	cout << "[Холмс]" << msg->data.c_str() << " " << msg->type << endl;
}

int main(int argc, char **argv)
{
	setlocale(0, "en_US.UTF-8");
	ros::init(argc, argv, "place");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("TrafalgarSquare", 1000, chatterCallback);
	ros::spin();

	return 0;
}