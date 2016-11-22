#include "ros/ros.h"
#include "place/Phrase.h"

#include <iostream>
#include <fstream>
#include <string>
using namespace std;

int main(int argc, char ** argv)
{
	setlocale(0, "en_US.UTF-8");
	ros::init(argc, argv, "holmes");

	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<place::Phrase>("TrafalgarSquare", 1000);
	ros::Rate loop_rate(10);

	ifstream fileName;
	fileName.open("phrases");

	while (pub.getNumSubscribers() == 0)
		sleep(1);

	while (ros::ok() && !fileName.eof())
	{
		place::Phrase msg;
		fileName >> msg.type;
		getline(fileName, msg.data);

		cout << msg.data.c_str() << endl;

		pub.publish(msg);

		ros::spinOnce();
		sleep(3);
	}

	return 0;
}