#include "ros/ros.h"
#include "man.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "girl");
	Man girl("Girl", "he");
	girl.start();
		
	return 0; 
}