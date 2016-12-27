#include "ros/ros.h"
#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>
#include "gazebo_msgs/ModelState.h"
#include "string.h"

using namespace ros;
using namespace tf;
using namespace std;

class Bot{
protected:
	NodeHandle 	node;
	Publisher 	pub;

	float 		x;
	float 		y;
	float 		w;
	string		my_name;

	TransformListener 		listener;
	TransformBroadcaster 	br;

	string				partner_name;
	StampedTransform 	partner_pos;
	bool				isPartnerOnMap;
	float				delta;

	void broadcastPose();
	bool isMet();
	void updatePartnerPos();

public:
	Bot();
	Bot(string my_name, string partner_name, float x, float y, float w);

	void updatePose();
};