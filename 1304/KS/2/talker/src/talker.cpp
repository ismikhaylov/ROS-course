#include "ros/ros.h"
#include "mess/Information.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tlk_confederate");
  ros::NodeHandle nodeHandle;
  ros::Publisher pub = nodeHandle.advertise<mess::Information>("scout", 1000);
  ros::Rate loop(1);

  while (ros::ok()) {
    mess::Information msg;
    msg.number = rand() % 100;
    msg.data = "Hello World";
    pub.publish( msg );
    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}