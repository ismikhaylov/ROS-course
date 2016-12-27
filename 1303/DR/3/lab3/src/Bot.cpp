#include "Bot.h"
#include <sstream>

Bot::Bot(NodeHandle & nodeHandler, int id, float x, float y)
	: id(id), rate(Rate(100)), nodeHandler(nodeHandler)
{
	this->x = x;
	this->y = y;
	factor = 100;

	rvizTopicName = "visualization_marker";
	rvizFrameId = "/bot_frame";
	rvizNamespace = "bots";

	ostringstream sout;
	sout << "/bot" << id << "/pose";
	poseTopicName = sout.str();

	rvizPublisher = nodeHandler.advertise<Marker>(rvizTopicName, 1);

	marker.header.frame_id = rvizFrameId;
	marker.header.stamp = Time::now();
	marker.ns = rvizNamespace;
	marker.id = id;
	marker.type = Marker::CUBE;
	marker.action = Marker::ADD;

	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;

	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = Duration();
}

void Bot::setColor(float r, float g, float b, float a)
{
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;

	repaint();
}

void Bot::publishPose()
{
	tf::Quaternion quaternion;
	quaternion.setRPY(0, 0, 0);

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x, y, 0));
	transform.setRotation(quaternion);

	transformBroadcaster.sendTransform(tf::StampedTransform(transform, Time::now(), "world", poseTopicName));
}

void Bot::repaint()
{
	marker.pose.position.x = x;
	marker.pose.position.y = y;

	rvizPublisher.publish(marker);
}

void Bot::goTo(float x, float y)
{
	float dx = fabs(this->x - x) / factor;
	dx *= this->x < x ? 1 : -1;

	float dy = fabs(this->y - y) / factor;
	dy *= this->y < y ? 1 : -1;

	for (float step = 0; step < factor && ok(); step++)
	{
		this->x += dx;
		this->y += dy;

		repaint();

		rate.sleep();
	}
	
	publishPose();
}