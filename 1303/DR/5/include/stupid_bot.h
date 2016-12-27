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
	bool rotated;
	const string selfTopicName;

	void followTheFriend();
	void runAround();
	void NewMessageCallback(const String::ConstPtr & msg);

public:
	StupidBot(NodeHandle & nodeHandler, float x = 0, float y = 0, int id = 0, int friendId = 1);

	void panic();
	void goTo(float x, float y, float angle = 0);
};

#endif // STUPID_BOT_H
