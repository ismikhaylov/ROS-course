//что-то вроде а давай знаешь, че мммммм, мммммммммммммммммм, ууууууууууууууууууу, хмхмхм, хххх давай напишем инклюд там ,че-нибудь
#include "robot.h"
#include <ctime>

int main(int argc, char **argv) {
	std::srand(unsigned(std::time(0)));
	ros::init(argc, argv, "robin");
	bool flag = false;
	Robot robin("robin", 1, 0.0, 1.0, 0.0, 0.0, 0.0);
	while (ros::ok()) {
		if (robin.isCloseEnough("batman") || flag)
		{	
			flag = true;
			robin.move(robin.m_anotherX, robin.m_anotherY);
			ROS_INFO ("robin move to batman (%f, %f)", robin.m_anotherX, robin.m_anotherY);
		}
		else
		{
			float x = std::rand() % 7;
			float y = std::rand() % 7; 
			robin.move(x, y);
			ROS_INFO ("robin move randomly (%f, %f)", x, y);
		}
		sleep(2);
	}
	return 0;
}