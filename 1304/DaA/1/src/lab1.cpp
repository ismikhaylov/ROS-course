#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ctime>

int main(int argc, char **argv)
{
  std::srand(0);
  ros::init(argc, argv, "lab1");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  ros::Rate loop_rate(10);
  
  while(ros::ok()) {
    geometry_msgs::Twist command;
    command.linear.x = std::rand() % 10;
    command.angular.z = std::rand() % 5;

    pub.publish(command);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}