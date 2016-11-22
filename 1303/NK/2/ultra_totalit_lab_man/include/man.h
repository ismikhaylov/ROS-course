#include <string>
#include <vector>
#include "ros/ros.h"
#include "ultra_totalit_lab_msg/tv_message.h"

using namespace std;

class Man{
	private:	
		string 				name;
		string 				partner_name;	
		vector<string> 		phrases;
		ros::NodeHandle 	node;
		bool 				isSpeaker;
	
		void init_phrases();
		void chatterCallback(const ultra_totalit_lab_msg::tv_message::ConstPtr& msg);	
		void start_speaker_part();
		void start_listener_part();
			
	public:
		Man();
		Man(string name, string partner_name);
			
		void start();
};