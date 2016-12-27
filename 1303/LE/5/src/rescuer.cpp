#include "Robot.h"
#include <stdlib.h>

int main( int argc, char** argv ) {
  ros::init(argc, argv, "rescuer");
  ros::NodeHandle n;
  ros::Rate r(2);
  ROS_INFO_STREAM("Wait");
  geometry_msgs::Pose pose;
  pose.position.x = -5;
  pose.position.y = 5;
  RescuerRobot rescuer(pose, n);
  ros::spin();
  return 0;
}