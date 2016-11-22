#include "ros/ros.h"
#include "sputnik_message/Coordinate.h"
#include <string>
#include <sstream>
#include <iostream>
#include <vector>

std::vector<int> coordinates;
void processReceiveData(const sputnik_message::Coordinate& msg)
{
  std::istringstream stream(msg.data);
  int coordinate;
  while (stream >> coordinate) {
	coordinates.push_back(coordinate);
  }
  if (coordinates.size() == 2) {
	std::cout << "sputnik_2: receive coordinates (x = " << coordinates[0] << ",y = " << 		coordinates[1] << ")" << std::endl;
  } else {
	std::cout << "sputnik_2: receive bad data" << std::endl;
  }
  coordinates.clear();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sputnik_2");
  ros::NodeHandle n;
  ros::Subscriber sp2_subscriber = n.subscribe("sputnik_msg", 5, processReceiveData);
  ros::spin();
  return 0;
}
