#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ctime>

int main(int argc, char **argv) {
std::srand(unsigned(std::time(0)));
ros::init(argc,argv,"turtle_mover");
ros::NodeHandle nh;
ros::Publisher pub = 
nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10,true);
for (int i = 0; i < 20; i++) {
	geometry_msgs::Twist msg;
	msg.linear.x = 1 + (std::rand() % 3);
	msg.angular.z = (std::rand() % 3) * ((std::rand() % 2) >0 ? 1: -1);
	pub.publish (msg);
	sleep(1);
}
return 0;
}