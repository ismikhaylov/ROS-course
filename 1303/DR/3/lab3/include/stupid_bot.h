#ifndef STUPID_BOT_H
#define STUPID_BOT_H

#include "Bot.h"
#include <cstdlib>
#include <ctime>
#include <sstream>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

using namespace std_msgs;

class StupidBot : public Bot
{
protected:
	int friendId;
	bool wasFound;
	string friendTopicName;
	tf::TransformListener listener;
	Subscriber subscriber;

	void followTheFriend();
	void runAround();
	void NewMessageCallback(const String::ConstPtr & msg);

public:
	StupidBot(NodeHandle & nodeHandler, int id = 0, int friendId = 1);

	void panic();
};

#endif // STUPID_BOT_H