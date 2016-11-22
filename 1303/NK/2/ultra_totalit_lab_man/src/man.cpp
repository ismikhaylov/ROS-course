#include "man.h"
#include <time.h> 

Man::Man(){
	Man("Speaker", "unknown");
}

Man::Man(string name, string partner_name){
	this->name = name;
	this->partner_name = partner_name;
	
	init_phrases();
	srand (time(NULL));
	isSpeaker = rand()%2;
}
	
void Man::init_phrases(){
	phrases.push_back("War is peace.");
	phrases.push_back("Freedom is slavery.");
	phrases.push_back("Ignorance is strength.");
	phrases.push_back("Big brother is watching you.");
	phrases.push_back("We shall meet in the place where there is no darkness.");
	
	phrases.push_back("In the face of pain there are no heroes.");
	phrases.push_back("It's a beautiful thing, the destruction of words.");
	phrases.push_back("Sanity is not statistical.");
	phrases.push_back("It's a beautiful thing, the destruction of words.");
	phrases.push_back("It was a bright cold day in April, and the clocks were striking thirteen.");
}

void Man::chatterCallback(const ultra_totalit_lab_msg::tv_message::ConstPtr& msg){	
	int id = msg->id;
	if(id % 5 == 0){
		ROS_INFO("%s: \"%s\" - %s said.",name.c_str(), msg->text.c_str(), partner_name.c_str());
	}
}

void Man::start_speaker_part(){
	ROS_INFO("%s: I'm a speaker!", name.c_str());
	ros::Publisher chatter_pub = node.advertise<ultra_totalit_lab_msg::tv_message>("tv", 1000);
	ros::Rate loop_rate(1);

	int i = 0;
	int ph_size = phrases.size();
	
	while (ros::ok()) {
		ultra_totalit_lab_msg::tv_message msg;
		
		msg.id = i + 1;
		msg.text = phrases[i];
		
		ROS_INFO("%s: %s", name.c_str(), phrases[i].c_str());
		chatter_pub.publish(msg);		
		
		i = (++i) % ph_size;
		loop_rate.sleep();
	}
}

void Man::start_listener_part(){
	ROS_INFO("%s: I'm watching tv!", name.c_str());
	ros::Subscriber sub = node.subscribe("tv", 1000, &Man::chatterCallback, this);
	ros::spin();
}

void Man::start(){
	if(isSpeaker){	
		start_speaker_part();		
	} else{
		start_listener_part();
	}
}