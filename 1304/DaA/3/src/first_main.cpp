#include "first_behavior.h"

int main( int argc, char** argv ){
	ros::init(argc, argv, "first");
	Bot *bot = new Bot(0, "first", "f_marker", -5.0, -5.0);
	bot->color(1.0, 0.0, 0.0);
	FirstBehavior *first = new FirstBehavior(bot, "second", "/lab3");
	first->run();

	return 0;
}