#include "Robot.h"
#include <stdlib.h>

int main( int argc, char** argv ) {
  ros::init(argc, argv, "killer");
  ros::NodeHandle n;
  ros::Rate r(2);
  geometry_msgs::Pose pose;

  tf::Quaternion q(tf::Vector3(0, 0, 1), M_PI);
  geometry_msgs::Quaternion odom_quat;
  tf::quaternionTFToMsg(q, odom_quat);
  pose.orientation = odom_quat;

  pose.position.x = 10.3;
  Killer killer(pose, n, 3);
  
  while (ros::ok()) {
    killer.followTarget();
    r.sleep();
  }
  return 0;
}