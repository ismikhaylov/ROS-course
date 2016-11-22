#include <ros/ros.h>
#include <ctime>
#include "rocket/rocket.h"

int main(int argc, char **argv) {
	std::srand(unsigned(std::time(0)));
	ros::init(argc, argv, "attacker");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<rocket::rocket>("/clear_sky", 10, true);
	
	while (true) {
		rocket::rocket rocket;
		rocket.x = std::rand() % 11;
		rocket.y = std::rand() % 11; 
		ROS_INFO ("Attacker sends rocket to (%d, %d)", rocket.x, rocket.y);
		pub.publish (rocket);
		sleep(5);
	}
	return 0;
}

