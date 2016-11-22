#ifndef BOT_H
#define BOT_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"

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
	float angle;

	string gazeboTopicName;
	string poseTopicName;

	NodeHandle & nodeHandler;
	Publisher gazeboPublisher;
	gazebo_msgs::ModelState msg;
	tf::TransformBroadcaster transformBroadcaster;
	Rate rate;

	void repaint();
	void publishPose();
	void setRPY(geometry_msgs::Pose & p, float roll, float pitch, float yaw);
	double getAngle(tf::Quaternion & q);

public:
	Bot(NodeHandle & nodeHandler, int id = 0, float x = 0, float y = 0);

	void goTo(float x, float y, float angle = 0);
	//void setColor(float r, float g, float b, float a = 1);
};

#endif // BOT_H