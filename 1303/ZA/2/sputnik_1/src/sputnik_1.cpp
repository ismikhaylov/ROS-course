#include "ros/ros.h"
#include "sputnik_message/Coordinate.h"
#include <time.h>
#include <math.h>

int main(int argc, char **argv)
{
  srand (time(NULL));
  ros::init(argc, argv, "sputnik_1");
  ros::NodeHandle n;
  ros::Publisher sp1_publisher = n.advertise<sputnik_message::Coordinate>("sputnik_msg", 5);
  ros::Rate loop_rate(10);

  sputnik_message::Coordinate msg;
  std::stringstream ss;
  while (ros::ok()) {
    ss.str("");
    ss << (rand()%90) << " " << (rand()%90);
    msg.data = ss.str();
    ROS_INFO("sputnik_1 send message %s", msg.data.c_str());
    sp1_publisher.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


