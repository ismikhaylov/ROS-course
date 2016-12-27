#include "ros/ros.h"
#include "place/Phrase.h"

#include <iostream>
#include <fstream>
#include <string>
using namespace std;

void chatterCallback(const place::Phrase::ConstPtr& msg)
{
	if (msg->type % 2 == 0)
		cout << "[Холмс]" << msg->data.c_str() << endl;
}

int main(int argc, char **argv)
{
	setlocale(0, "en_US.UTF-8");
	ros::init(argc, argv, "watson");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("TrafalgarSquare", 1000, chatterCallback);
	cout << "[Ватсон] Жду Холмса" << endl;
	ros::spin();

	return 0;
}