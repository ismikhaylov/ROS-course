#include "ros/ros.h"
#include "Robot.h"

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "missing");

	ros::Time::init();
	ros::Rate loop(1);

	Robot *mR = new Robot( "missing", 0 );

	while (ros::ok()) {

		if (mR->checkPosAnotherRobot( "assistant" )) {
			mR->moveToAnotherRobot();
		} else {			
			mR->move( rand() % 5, rand() % 5 );
		}
	}
	return 0;
}