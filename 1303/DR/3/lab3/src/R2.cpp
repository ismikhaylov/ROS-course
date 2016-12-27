#include "stupid_bot.h"

int main( int argc, char** argv )
{
	init(argc, argv, "StupidBot");
	NodeHandle nodeHandle;

	StupidBot bot(nodeHandle);
	bot.panic();

	return 0;
}