#include "smart_bot.h"

SmartBot::SmartBot(NodeHandle & nodeHandler, float x, float y, int id, int friendId)
  : Bot(nodeHandler, id, x, y)
{
  this->friendId = friendId;
  startPoint.x = x;
  startPoint.y = y;

  ostringstream sout;
  sout << "/bot" << friendId << "/pose";
  friendTopicName = sout.str();

  sout.str("");
  sout.clear();

  sout << "/bot" << friendId << "/cmds";
  publisher = nodeHandler.advertise<String>(sout.str(), 10);

  setColor(0, 1, 0.5);
}

bool SmartBot::isFoundAt(float x, float y)
{
  float dx = fabs(this->x - x);
  float dy = fabs(this->y - y);

  return dx < 0.5 && y < 0.5;
}

void SmartBot::findTheFriend()
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

    if (isFoundAt(transform.getOrigin().x(), transform.getOrigin().x()))
    {
      String msg;
      msg.data = "found";

      publisher.publish(msg);
      return;
    }

    goTo(transform.getOrigin().x(), transform.getOrigin().y());
    rate.sleep();
  }
}

void SmartBot::goBack()
{
  goTo(startPoint.x, startPoint.y);
}
