#include "Sense.h"
#include <stdlib.h>

int main( int argc, char** argv ) {
  ros::init(argc, argv, "sense");
  ros::NodeHandle n;
  ros::Rate r(10);
  geometry_msgs::Pose pose;

  pose.position.x = -10.0;
  pose.position.y = -10.0;

  geometry_msgs::Pose poseStart;
  poseStart.position.x = 10.3;

  Sense sense({20, 20, 3, pose}, poseStart, 3, n);
  sense.generateTargets(3);
  sense.generateBarriers(10);

  while (!sense.end) {
    sense.work();
  }
  return 0;
}