#include "ros/ros.h"
#include "lover_msg/loverMesg.h"


void chatterCallback(const lover_msg::loverMesg& msg)
{
  if((msg.x)%5==0)
    ROS_INFO("I heard: [%s]", msg.a.c_str());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();

  return 0;
}
