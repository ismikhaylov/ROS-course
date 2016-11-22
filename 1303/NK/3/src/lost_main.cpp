#include "tf_lab/lostbot.h"

int main( int argc, char** argv ){
	ros::init(argc, argv, "lost");
	LostBot lost("saver", "lost", "lost_marker", 2, -2);
	lost.setColor(0.9, 0.3, 0.5);
	lost.start();
	return 0;
}