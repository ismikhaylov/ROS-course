#include "ros/ros.h"
#include "man.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "guy");
	Man guy("Guy", "she");
	guy.start();
		
	return 0; 
}