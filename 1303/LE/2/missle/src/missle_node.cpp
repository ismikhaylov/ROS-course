#include "ros/ros.h"
#include "position_msg/Position.h"
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "painter");
  ros::NodeHandle nodeHandle;

  if (argc < 3) {
    ROS_ERROR("It's necessary to set position of missle fall.");
    return 1;
  }

  const int queueSize = 1000;
  ros::Publisher publisher = nodeHandle.advertise<position_msg::Position>("missle_pos", queueSize);
  ros::Rate rate(1);
  position_msg::Position msg;
  msg.x = std::stof(argv[1]);
  msg.y = std::stof(argv[2]);

  publisher.publish(msg);
  rate.sleep();
  publisher.publish(msg);
  rate.sleep();
  return 0;
}