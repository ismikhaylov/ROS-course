//что-то вроде а давай знаешь, че мммммм, мммммммммммммммммм, ууууууууууууууууууу, хмхмхм, хххх давай напишем инклюд там ,че-нибудь
#include "robot.h"
#include <ctime>

int main(int argc, char **argv) {
	std::srand(unsigned(std::time(0)));
	ros::init(argc, argv, "batman");
	bool flag = false;
	Robot batman("batman", 1, 0.0, 0.0, 0.0, 10.0, 10.0);
	while (ros::ok()) {
		if (batman.isCloseEnough("robin") || flag)
		{	
			flag = true;
			batman.move(10.0, 10.0);
			ROS_INFO ("batman move home (%f, %f)", 10.0, 10.0);
		}
		else
		{
			batman.move(batman.m_anotherX, batman.m_anotherY);
			ROS_INFO ("batman move to robin (%f, %f)", batman.m_anotherX, batman.m_anotherY);
		}
		sleep(2);
	}
	return 0;
}