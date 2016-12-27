#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "run_turtle_run");
  ROS_INFO_STREAM("The run_turtle_run node is ready!");
  
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  
  geometry_msgs::Twist command;
  command.linear.x = 4.0;
  command.angular.z = -1.8;
  
  while(ros::ok()) {
	pub.publish(command);
	ros::spinOnce();
  }
  return 0;
}
