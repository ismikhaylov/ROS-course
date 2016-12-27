#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_broadcaster.h>
#include <string> 

using namespace ros;
using namespace tf;
using namespace visualization_msgs;
using namespace std;

class Bot {
protected:
	float 		speed;
	NodeHandle 	node;
	Publisher 	pub;
	Marker 		marker;

	string		rviz_marker_topic;
	string		rviz_frame_id;
	string		rviz_namespace;
	string		base_frame_id;
	string		child_frame_id;

	TransformBroadcaster	br;

	void draw();
	void broadcast();
public:
	Bot(int id, string name, string rviz_marker_topic, float x, float y);
	void move(float x, float y);
	void color(float r, float g, float b);
	float getX();
	float getY();

	NodeHandle* getNode();
};

bool move(Bot *bot, float target_x, float target_y);
bool listen_position(string frame_id, string target, TransformListener& listener, Vector3& result);
bool collision(float x1, float y1, float x2, float y2, float delta);