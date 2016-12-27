#include "stupid_bot.h"

StupidBot::StupidBot(NodeHandle & nodeHandler, int id, int friendId)
	: Bot(nodeHandler, id)
{
	this->friendId = friendId;
	wasFound = false;

	ostringstream sout;
	sout << "/bot" << friendId << "/pose";
	friendTopicName = sout.str();

	sout.str("");
	sout.clear();

	sout << "/bot" << id << "/cmds";
	subscriber = nodeHandler.subscribe(sout.str(), 10, &StupidBot::NewMessageCallback, this);

	setColor(1, 0.3, 0.2);
}

void StupidBot::NewMessageCallback(const String::ConstPtr & msg)
{
	wasFound = msg->data == "found";

	if (wasFound)
		setColor(0, 1, 0);
}

void StupidBot::followTheFriend()
{
	listener.waitForTransform("/world", friendTopicName, Time::now(), Duration(1));

	Rate rate(10);

	while (nodeHandler.ok())
	{
		tf::StampedTransform transform;

		try {
			listener.lookupTransform("/world", friendTopicName, Time(0), transform);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s", ex.what());
			sleep(1);
			continue;
		}

		goTo(transform.getOrigin().x(), transform.getOrigin().y());
		rate.sleep();
	}
}

void StupidBot::runAround()
{
	int dx = rand() % 10;
	int dy = rand() % 10;

	dx *= rand() % 2 == 0 ? 1 : -1;
	dy *= rand() % 2 == 0 ? 1 : -1;

	goTo(x + dx, y + dy);
}

void StupidBot::panic()
{
	int k = 0;

	while (nodeHandler.ok())
	{
		if (wasFound)
		{
			followTheFriend();
			return;
		}

		spinOnce();
		if (k < 15)
		{
			runAround();
			k++;
		}
	}
}