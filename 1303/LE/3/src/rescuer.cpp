#include "Robot.h"
#include <stdlib.h>

int main( int argc, char** argv ) {
  ros::init(argc, argv, "rescuer");
  ros::NodeHandle n;
  ros::Rate r(2);
  ROS_INFO_STREAM("Wait");
  RescuerRobot rescuer({-5,-5}, n);
  ros::spin();
  return 0;
}