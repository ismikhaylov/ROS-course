#include "ros/ros.h"
#include "Robot.h"

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "assistant");

	ros::Time::init();
	ros::Rate loop(1);

	Robot *mR = new Robot( "assistant", 1 );

	while (ros::ok()) {

		if (!mR->checkPosAnotherRobot("missing")) {
			mR->moveToAnotherRobot();
		} else {
			mR->move( 0, 0 );
		}
	}
	return 0;
}