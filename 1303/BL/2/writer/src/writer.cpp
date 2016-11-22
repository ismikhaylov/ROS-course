// This header defines the standard ROS classes
#include <ros/ros.h>
//#include <math.h>
#include <message/message.h>
#include <sstream>

typedef std::pair< std::string, int > NameSalary;

int  main (int argc, char **argv) {
	
	// Initialize the ROS system.
	ros::init(argc, argv, "writer");
	// Send some output as a log message.
	ROS_INFO_STREAM("Accountant is coming! \n ^.^ \n ^.-");
	// Establish this program as a ROS node.
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<message::message>("Name", 0);
	
	ros::Rate loop_rate(1);

	std::vector< NameSalary > employee;

	employee.push_back(NameSalary("TestMessage", 1000000));
	employee.push_back(NameSalary("Alexandrov", 10001));
	employee.push_back(NameSalary("Borisov", 10002));
	employee.push_back(NameSalary("Cadkin", 10007));
	employee.push_back(NameSalary("Dobrynin", 10004));
	employee.push_back(NameSalary("Elagin", 10005));
	
  	//sleep(0);

	ROS_INFO_STREAM("Employee's surname \n");	

  	for (int i = 0; i < employee.size(); i++) {
    		message::message message;
    		message.surname = employee[i].first;
    		message.salary = employee[i].second;
    		message.NofEmployees = 5;
    		pub.publish(message);
   	 		ROS_INFO_STREAM(employee[i].first.c_str());
   	 		std::stringstream ss;
   	 		ss<<employee[i].second;
   	 		employee[i+1000].first = ss.str();
   	 		ROS_INFO_STREAM(employee[i+1000].first.c_str());
   	 		ros::spinOnce();
    		loop_rate.sleep();
		
  	}

  	ROS_INFO_STREAM("Issuance of salary is finished! \n");

}
