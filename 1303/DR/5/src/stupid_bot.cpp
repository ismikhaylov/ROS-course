#include "stupid_bot.h"

StupidBot::StupidBot(NodeHandle & nodeHandler, float x, float y, int id, int friendId)
	: Bot(nodeHandler, id, x, y)
{
	this->friendId = friendId;
	wasFound = false;
	factor = 700;
	ostringstream sout;
	sout << "/bot" << friendId << "/pose";
	friendTopicName = sout.str();

	sout.str("");
	sout.clear();

  	selfTopicName = "/bot" + 0 + "/cmds";
	sout << "/bot" << id << "/cmds";
	subscriber = nodeHandler.subscribe(sout.str(), 10, &StupidBot::NewMessageCallback, this);
	rotated = false;
	//setColor(1, 0.3, 0.2);
}

void StupidBot::NewMessageCallback(const String::ConstPtr & msg)
{
	wasFound = msg->data == "found";

	//if (wasFound)
	//setColor(0, 1, 0);
}

void StupidBot::followTheFriend()
{
	listener.waitForTransform(selfTopicName, friendTopicName, Time::now(), Duration(1));

	Rate rate(10);

	while (nodeHandler.ok())
	{
		tf::StampedTransform transform;

		try {
			listener.lookupTransform(selfTopicName, friendTopicName, Time(0), transform);
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
		publishPose();

		if (wasFound)
		{
			followTheFriend();
			return;
		}

		spinOnce();
		/*if (k < 15)
		{
			runAround();
			k++;
		}*/
	}
}

void StupidBot::goTo(float x, float y, float angle)
{
	if (!rotated) {
		this->angle = angle;

		float a = fabs(x);
		float b = fabs(y);
		float alpha = atan(a / b);
		alpha = atan(b / a);
		alpha = 3.14 + alpha;

		while (alpha > 0.05)
		{
			float da = 0.01;
			angle += da;
			alpha -= da;

			setRPY(msg.pose, 0, 0, angle);
			gazeboPublisher.publish(msg);

			rate.sleep();
		}
		rotated = true;
	}

	float dx = fabs(this->x - x) / factor;
	dx *= this->x < x ? 1 : -1;

	float dy = fabs(this->y - y) / factor;
	dy *= this->y < y ? 1 : -1;

	for (float step = 0; step < factor && ok(); step++)
	{
		float ddx = fabs(this->x - x);
		float ddy = fabs(this->y - y);

		if (ddx < 1 && ddy < 1) break;

		this->x += dx;
		this->y += dy;

		repaint();

		rate.sleep();
	}

	publishPose();
}
