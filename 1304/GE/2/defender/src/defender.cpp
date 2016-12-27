#include <ros/ros.h>
#include <ctime>
#include "rocket/rocket.h"

void rocket_handler (const rocket::rocket& msg) {
	rocket::rocket t1, t2, t3;
	t1.x = std::rand() % 11;	
	t1.y = std::rand() % 11;	
	t2.x = std::rand() % 11;	
	t2.y = std::rand() % 11;	
	t3.x = std::rand() % 11;	
	t3.y = std::rand() % 11;
	
	int chance = 0;
	if ((msg.x == t1.x && msg.y == t1.y) ||
	    (msg.x == t2.x && msg.y == t2.y) ||
	    (msg.x == t3.x && msg.y == t3.y)) {
		chance = 80;
	} else {
		chance = 5;
	}

	if ((std::rand() % 100) < chance) {
		ROS_INFO("Defeated!");
	} else {
		ROS_INFO("BOOM!");
	}
}

int main(int argc, char **argv) {
        std::srand(unsigned(std::time(0)));
        ros::init(argc, argv, "defender");
        ros::NodeHandle nh;
        ros::Subscriber sub = nh.subscribe("/clear_sky", 10, &rocket_handler);
	ros::spin();
        return 0;
}

