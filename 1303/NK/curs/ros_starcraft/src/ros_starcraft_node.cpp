#include "ros/ros.h"
#include "Configs.h"
#include "World.h"

int main( int argc, char** argv ){
	ros::init(argc, argv, "ros_starcraft");
	NodeHandle node;

	Configs cnf;
	cnf.parse();

	World wld(node);
	wld.start();
	
	return 0;
}