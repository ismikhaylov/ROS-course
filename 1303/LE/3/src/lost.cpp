#include "Robot.h"
#include <stdlib.h>

int main( int argc, char** argv ) {
  ros::init(argc, argv, "lost");
  ros::NodeHandle n;
  ros::Rate r(10);
  int k = 0;
  SimpleRobot lost({4,4}, n);
  lost.sendHelpMessage();
  while (ros::ok()) {
    if (lost.isSaved()) {
        return 0;
    }
    ros::spinOnce();
    if (!lost.isSaved()) {
        Position curPosition = lost.getPosition();
        int x = rand() % 5;
        int y = rand() % 5;
        x = x * (rand() % 2 ? 1 : -1);
        y = y * (rand() % 2 ? 1 : -1);
        lost.run({curPosition.x + x, curPosition.y + y});
        r.sleep();
        k++;
    }
  }
  ros::spin();
  return 0;
}