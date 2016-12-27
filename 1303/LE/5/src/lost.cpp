#include "Robot.h"
#include <stdlib.h>

int main( int argc, char** argv ) {
  ros::init(argc, argv, "lost");
  ros::NodeHandle n;
  ros::Rate r(10);
  geometry_msgs::Pose pose;
  SimpleRobot lost(pose, n);
  
  lost.sendHelpMessage();
  while (ros::ok()) {
    if (lost.isSaved()) {
        return 0;
    }
    ros::spinOnce();
    if (!lost.isSaved()) {
        geometry_msgs::Pose curPosition = lost.getPosition();
        int x = rand() % 2 + 1;
        int y = rand() % 2 + 1;
        x = x * (rand() % 2 ? 1 : -1);
        y = y * (rand() % 2 ? 1 : -1);
        geometry_msgs::Pose pose;
        pose.position.x = curPosition.position.x + x;
        pose.position.y = curPosition.position.y + y;
        lost.run(pose);
        r.sleep();
    }
  }
  return 0;
}