#include "ros/ros.h"
#include "speaker/Speach.h"

#include <iostream>
#include <fstream>
#include <string>
using namespace std;

int lexa = 0;

void chatterCallback(const speaker::Speach::ConstPtr& msg)
{
	if ((lexa++) % 5 == 0) {
	        ROS_INFO(("Вас понял: " + msg->text).c_str());
	}
}

int main(int argc, char **argv)
{
        setlocale(0, "en_US.UTF-8");
        ros::init(argc, argv, "friend");

        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("Performance", 1000, chatterCallback);
        ros::spin();

        return 0;
}

