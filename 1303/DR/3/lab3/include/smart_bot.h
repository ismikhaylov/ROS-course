#ifndef SMART_BOT_H
#define SMART_BOT_H

#include "Bot.h"
#include <cstdlib>
#include <ctime>
#include <sstream>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

using namespace std_msgs;

class SmartBot : public Bot
{
protected:
	int friendId;
	string friendTopicName;
	tf::TransformListener listener;
	Publisher publisher;
	Point startPoint;

	bool isFoundAt(float x, float y);

public:
	SmartBot(NodeHandle & nodeHandler, float x = 0, float y = 0, int id = 1, int friendId = 0);

	void findTheFriend();
	void goBack();
};

#endif // SMART_BOT_H