#include <ros/ros.h>
#include <ros/console.h>
#include <messages/payroll.h>
#include <std_msgs/Int8.h>

int count = -1;
int maxSalary = -1;

void handler(const messages::payroll& msg)
{
	if (msg.salary > maxSalary)
	{
		maxSalary = msg.salary;
		ROS_INFO("%s %s says \'kek\', his salary: %d", msg.firstname.c_str(), msg.lastname.c_str(), msg.salary);
	}
	if (--count == 0) 
	{
		ROS_INFO("All employee are get salary.");
		ros::shutdown();
	}
}

void handlerSize(const std_msgs::Int8& c)
{
  	count = c.data;
  	ROS_INFO("Queue size: %d", count);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "employee");
  ROS_INFO("The employee is ready!");
    
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/squeue", 100, &handlerSize);
  
  ros::NodeHandle nh2;
  ros::Subscriber sub2 = nh2.subscribe("/salary", 100, &handler);
  
  ros::spin();    
  return 0;
}



