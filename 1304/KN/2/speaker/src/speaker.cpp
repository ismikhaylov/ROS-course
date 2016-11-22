#include "ros/ros.h"
#include "speaker/Speach.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
using namespace std;

static const std::string arr[] = {
	"Скамейка в парке прогнила",
	"На автобусной остановке забыли сумку.",
	"В школе прошёл детский утренник",
	"Ко мне в гости приехал брат",
	"На пощади Ленина состоялся фестиваль кепок",
	"От Волги до Енисея - Россия, моя Россия"
};
vector<std::string> vec (arr, arr + sizeof(arr) / sizeof(arr[0]) );

void loop(ros::Publisher& pub);

int main(int argc, char ** argv)
{
        setlocale(0, "en_US.UTF-8");
        ros::init(argc, argv, "speaker");

        ros::NodeHandle n;
        ros::Publisher pub = n.advertise<speaker::Speach>("Performance", 1000);
        ros::Rate loop_rate(100);

        while (ros::ok())
        {
                loop(pub);
        }

        return 0;
}

void loop(ros::Publisher& pub) {

	for (int i = 0; i < vec.size(); i++) {
    		speaker::Speach msg;
		msg.text = vec[i];
		ROS_INFO(msg.text.c_str());
		pub.publish(msg);
		ros::spinOnce();
		sleep(3);
	}
}

