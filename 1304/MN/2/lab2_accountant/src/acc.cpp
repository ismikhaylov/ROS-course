#include <ros/ros.h>
#include <ros/console.h>
#include <lab2_msg/money.h>
#include <vector>
#include <string>
#include <std_msgs/Int8.h>

struct Money
{
    std::string name;
    std::string lastname;
    int money;

    Money(const std::string& name, const std::string& lastname, int money)
    {
        this->name = name;
        this->lastname = lastname;
        this->money = money;
    }
};

void notify(int size)
{
	std_msgs::Int8 s;
	s.data = size;
	ros::NodeHandle nh;
  	ros::Publisher pub = nh.advertise<std_msgs::Int8>("/squeue", 100);
  	sleep(3);
  	
  	pub.publish(s);
  	ros::spinOnce();
  	ROS_INFO("Employee queue size: %d", s.data);
}

void init_payment_list(std::vector<Money>& payments) 
{
  payments.push_back(Money("Will", "Smith", 2900));
  payments.push_back(Money("Petr", "Osetr", 1000));
  payments.push_back(Money("Ivan", "Kartofan", 3500));
  payments.push_back(Money("Fedor", "Lopata", 1600));
  payments.push_back(Money("Harry", "Potter", 2500));
  payments.push_back(Money("Alisa", "Selezneva", 1100));
  payments.push_back(Money("Fedor", "Dvinyatin", 4600));
  payments.push_back(Money("Bruce", "Boltrashevich", 2500));
  payments.push_back(Money("John", "TheAmerican", 3100));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "accountant");
  ROS_INFO("Hello! I`m an accountant!");
    
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<lab2_msg::money>("/money", 100);
  sleep(3);
  
  ros::Rate loop_rate(1);
  std::vector<Money> payments;
  init_payment_list(payments);
  
  notify(payments.size());
    
  lab2_msg::money budget;
  for (int i = 0; i < payments.size(); i++)
    {
	budget.name = payments[i].name;
	budget.lastname = payments[i].lastname;
	budget.money = payments[i].money;
        pub.publish(budget);
        loop_rate.sleep();
    }
  
  ROS_INFO("The accountant send all payments.");
  return 0;
}


