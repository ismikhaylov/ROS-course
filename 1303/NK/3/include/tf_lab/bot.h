#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_broadcaster.h>
#include <string> 

using namespace ros;
using namespace tf;
using namespace visualization_msgs;
using namespace std;

class Bot{
protected:
	static int 	count;
	
	int			id;
	NodeHandle 	node;
	Publisher 	pub;
	Marker 		marker;

	string		rviz_marker_topic;
	string		rviz_frame_id;
	string		rviz_namespace;
	string		base_frame_id;
	string		child_frame_id;

	TransformListener 		listener;
	TransformBroadcaster	br;

	string				partner_name;
	StampedTransform 	partner_pos;
	float				delta;

	void repaint();
	void broadcastPose();
	bool isMet();
	void updatePartnerPos();

public:
	Bot();
	Bot(string partner_name, string name, string rviz_marker_topic, float x, float y);

	void move(float x, float y);
	void setColor(float r, float g, float b);
	float getX();
	float getY();

};