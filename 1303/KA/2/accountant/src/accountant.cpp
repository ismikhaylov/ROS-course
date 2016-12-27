#include <ros/ros.h>
#include <ros/console.h>
#include <messages/payroll.h>
#include <vector>
#include <string>
#include <std_msgs/Int8.h>

struct Payroll
{
    std::string firstname;
    std::string lastname;
    int salary;

    Payroll(const std::string& firstname, const std::string& lastname, int salary)
    {
        this->firstname = firstname;
        this->lastname = lastname;
        this->salary = salary;
    }
};

void informQueue(int size)
{
	std_msgs::Int8 s;
	s.data = size;
	ros::NodeHandle nh;
  	ros::Publisher pub = nh.advertise<std_msgs::Int8>("/squeue", 100);
  	sleep(3);
  	
  	pub.publish(s);
  	ros::spinOnce();
  	ROS_INFO("Queue size: %d", s.data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "accountant");
  ROS_INFO("The accountant is ready!");
    
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<messages::payroll>("/salary", 100);
  sleep(3);
  
  ros::Rate loop_rate(1);
  std::vector<Payroll> pays;
  pays.push_back(Payroll("Andrey", "Popov", 1700));
  pays.push_back(Payroll("Petr", "Petrov", 2000));
  pays.push_back(Payroll("Serge", "Sidorov", 1500));
  pays.push_back(Payroll("Ivan", "Ivanov", 1600));
  pays.push_back(Payroll("Alexander", "Kotov", 2200));
  pays.push_back(Payroll("Evgeny", "Smirnov", 2100));
  
  informQueue(pays.size());
    
  messages::payroll p;
  for (int i = 0; i < pays.size(); i++)
    {
		p.firstname = pays[i].firstname;
		p.lastname = pays[i].lastname;
		p.salary = pays[i].salary;
        pub.publish(p);
        loop_rate.sleep();
    }
  
  ROS_INFO("All salaries are paid by accountant.");
  return 0;
}


