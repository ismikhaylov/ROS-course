#include <ros/ros.h>
#include <ros/console.h>
#include <lab2_msg/money.h>
#include <std_msgs/Int8.h>

int count = -1;
int big_money = -1;

void handler(const lab2_msg::money& msg)
{
	if (msg.money > big_money)
	{
		big_money = msg.money;
		ROS_INFO("%s %s :: oh my money! %d founds are so much!", msg.name.c_str(), msg.lastname.c_str(), msg.money);
	}
	if (--count == 0) 
	{
		ROS_INFO("All employee are get money.");
		ros::shutdown();
	}
}

void handlerSize(const std_msgs::Int8& c)
{
  	count = c.data;
  	ROS_INFO("Employee queue size: %d", count);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "employee");
  ROS_INFO("I`m an employee");
    
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/squeue", 100, &handlerSize);
  
  ros::NodeHandle nh2;
  ros::Subscriber sub2 = nh2.subscribe("/money", 100, &handler);
  
  ros::spin();    
  return 0;
}



