#include "second_behavior.h"

int main( int argc, char** argv ) {
	ros::init(argc, argv, "second");
	Bot *bot = new Bot(1, "second", "s_marker", 0.0, 0.0);
	bot->color(0.0, 1.0, 0.0);
	SecondBehavior *second = new SecondBehavior(bot, "first", "/lab3");
	second->run();

	return 0;
}