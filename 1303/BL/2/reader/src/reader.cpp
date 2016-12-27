#include <ros/ros.h>
#include <message/message.h>
#include <math.h>

std::string surname;
message::message::_salary_type maxSalary = 0;
message::message::_NofEmployees_type K = 0;

void reader (const message::message & message)
{
  message::message::_salary_type salary = message.salary;
  K++;
  ROS_INFO("%s is listening  (0_0) ", message.surname.c_str());

  if(salary > maxSalary) {
    maxSalary = salary;
    surname = message.surname;
  }

  if(K == message.NofEmployees) {
      ROS_INFO("\nWin!!! ^.^ I am %s. My salary is %d $. \n", surname.c_str(), maxSalary);
      ros::shutdown();
  }
}

int  main (int argc, char **argv) {
	
	// Initialize the ROS system.
	ros::init(argc, argv, "reader");
	// Send some output as a log message.
	ROS_INFO_STREAM("Employees are coming! \n ^.- \n ^_^ ");
	// Establish this program as a ROS node.
	ros::NodeHandle n;

	std::string NodeName = ros::this_node::getName();
	surname = NodeName.substr(1, NodeName.length() - 1);

	ros::Subscriber sub = n.subscribe("Name", 0, reader);
	ros::spin();
    return 0;
}