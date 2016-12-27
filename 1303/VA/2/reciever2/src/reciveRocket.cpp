#include "ros/ros.h"
#include "std_msgs/String.h"
#include <roc_message/Messager.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <sstream>
#include <time.h>

using namespace std;

void chatterCallback(const roc_message::Messager::ConstPtr& msg)
{
  srand(time(NULL)-1000);
  int forRand = 1 + rand() % 3;
  cout<<"\nProtection zone is set to "<<forRand<<endl;
 

  if((msg->data) == "1"){
	cout<<"Rocket flying into the zone 1"<<endl;
	int zone = 1 + rand() % 100;
	if(forRand == 1){
		std::cout<< (zone >= 80 ? "Target has been destroyed" : "Target is protected") << endl;
	}
	else{
		std::cout<< (zone >= 5 ? "Target has been destroyed" : "Target is protected") << endl;	
	}
	
}
  if((msg->data) == "2"){
	cout<<"Rocket flying into the zone 2"<<endl;
	int zone = 1 + rand() % 100;
	if(forRand == 1){
		std::cout<< (zone >= 80 ? "Target has been destroyed" : "Target is protected") << endl;
	}
	else{
		std::cout<< (zone >= 5 ? "Target has been destroyed" : "Target is protected") << endl;	
	}
	
}
  if((msg->data) == "3"){
	cout<<"Rocket flying into the zone 3"<<endl;
	int zone = 1 + rand() % 100;
	if(forRand == 1){
		std::cout<< (zone >= 80 ? "Target has been destroyed" : "Target is protected") << endl;
	}
	else{
		std::cout<< (zone >= 5 ? "Target has been destroyed" : "Target is protected") << endl;	
	}
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reciveRocket");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
  return 0;
}
