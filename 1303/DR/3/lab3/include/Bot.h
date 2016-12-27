#ifndef BOT_H
#define BOT_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>

using namespace ros;
using namespace std;
using namespace visualization_msgs;
using namespace geometry_msgs;

class Bot
{
protected:
	const int id;

	float x;
	float y;
	float factor;

	string rvizTopicName;
	string rvizFrameId;
	string rvizNamespace;
	string poseTopicName;

	NodeHandle & nodeHandler;
	Publisher rvizPublisher;
	Marker marker;
	tf::TransformBroadcaster transformBroadcaster;
	Rate rate;

	void repaint();
	void publishPose();

public:
	Bot(NodeHandle & nodeHandler, int id = 0, float x = 0, float y = 0);
	
	void goTo(float x, float y);
	void setColor(float r, float g, float b, float a = 1);
};

#endif // BOT_H