#include "ros/ros.h"
#include "robot.h"

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "assistant");

	ros::Time::init();
	Robot *mR = new Robot( "assistant" );

	while (ros::ok()) {

		if (!mR->checkPosAnotherRobot( "missing" )) {
			mR->moveToAnotherRobot(3);
		} else {
			mR->move( -20, -20, 1.2 );
			if (mR->isPosition(-20, -20)) {
				ROS_INFO( "Assistant robot end." );
			}
		}
	}
	return 0;
}