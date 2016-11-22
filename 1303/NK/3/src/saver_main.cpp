#include "tf_lab/savebot.h"

int main( int argc, char** argv ){
	ros::init(argc, argv, "saver");
	SaveBot saver("lost", "saver", "saver_marker", -5, -5);
	saver.setColor(0.1, 0.6, 1.0);
	saver.start();
	return 0;
}