#include <ros/ros.h>
#include "packmsg/Secret_coords.h"
#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <cstring>


using namespace std;

 string decode(string pText, string pKey) {
	 	//std::stringstream ss2; //new
		char* res = new char[pText.length()];
		const char* key = pKey.c_str();

		for (int i = 0; i < pText.length(); i++) {
			res[i] = (char) (pText[i] ^ key[i % pKey.length()]);
			//ss2<<(char)(pText[i] ^ key[i % pKey.length()]);
		}

		return string(res);
	    //return ss2.str();
	}

void descrypt(const packmsg::Secret_coords& msgIn){
	
  string key = "Key_of_encrypt_machine_2016_";
  string descrypted_m = decode(msgIn.message, key);   
	ROS_INFO_STREAM("Descrypted: " <<descrypted_m);
}

int main(int argc, char ** argv)
{
   ros::init(argc, argv, "sSat");
   ros::NodeHandle nh;
   
   ros::Subscriber sub1 = nh.subscribe("satTopic", 1000, &descrypt);
   ros::spin();
   
   return 0;
}
