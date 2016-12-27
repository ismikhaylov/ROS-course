#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher topic=n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);

  ros::Rate loop_rate(10);
  geometry_msgs::Twist pos;

pos.linear.x=2.0;

pos.angular.z=-1.8;

int count=0;
while (count<100)
  {
ROS_INFO("Move to position:\n"
         "1) pos.linear: x= %f y= %f z=%f\n"
         "2) pos.angular: x=%f y=%f z= %f\n",
         pos.linear.x,pos.linear.y,pos.linear.z,
         pos.angular.x,pos.angular.y,pos.angular.z);
topic.publish(pos);
loop_rate.sleep();
count++;
}
  return 0;
}
