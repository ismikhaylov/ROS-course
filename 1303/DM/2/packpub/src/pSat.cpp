#include "ros/ros.h"
#include "packmsg/Secret_coords.h"
#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <cstring>


using namespace std;

 string encode(string & pText, string & pKey) {
	    //std::stringstream ss2; //new
		const char *txt = pText.c_str();
		const char *key = pKey.c_str();
		char *res = new char[pText.length()];

		for (int i = 0; i < pText.length(); i++) {
			res[i] = (char) (txt[i] ^ key[i % pKey.length()]);
			//ss2<< (txt[i] ^ key[i % pKey.length()]);
			
		}
		//ROS_INFO_STREAM("Res: " <<ss2.str());
		return string(res);
		//return ss2.str();
	}

void encrypt(float lat, float longi, packmsg::Secret_coords & msg )
{
	std::stringstream ss;
	ss << "latitude: " << lat << " longitude: " << longi;
	std::string st =ss.str();
	string key = "Key_of_encrypt_machine_2016_";
	
	msg.message = encode(st,key);
	ROS_INFO_STREAM("Message: " <<st);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pSat");
	ros::NodeHandle nh;
	
	ros::Publisher Pub = nh.advertise<packmsg::Secret_coords>("satTopic",1000);
	ros::Rate loop_rate(1);
	
	int i=0;
	srand(time(NULL));
	while(ros::ok())
	{
		if (i>359) i=0;
		packmsg::Secret_coords msg;
		
		float lat = rand()%200;
		float longi = rand()%200;
		encrypt( lat, longi, msg);
		
		
		Pub.publish(msg);
		ros::spinOnce();
	    loop_rate.sleep();
	    i++;	
	}
	
	return 0;
}
