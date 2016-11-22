#include "Bot.h"
#include <sstream>
#include <fstream>

Bot::Bot(NodeHandle & nodeHandler, int id, float x, float y)
	: id(id), rate(Rate(100)), nodeHandler(nodeHandler)
{
	this->x = x;
	this->y = y;
	factor = 1000;

	gazeboTopicName = "gazebo/set_model_state";

	ostringstream sout;
	sout << "/bot" << id << "/pose";
	poseTopicName = sout.str();

	gazeboPublisher = nodeHandler.advertise<gazebo_msgs::ModelState>(gazeboTopicName, 1);

	ros::service::waitForService("gazebo/spawn_sdf_model");
	ros::ServiceClient add_robot =
	    nodeHandler.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	gazebo_msgs::SpawnModel srv;

	string model;
	string buf;

	ifstream fin("/home/rina/.gazebo/models/pioneer2dx/model.sdf");


	while (!fin.eof() && fin) {
		getline(fin, buf);
		model += buf + "\n";
	}

	ostringstream id_str;
	id_str << "Bot" << id;

	srv.request.model_xml = model;
	srv.request.model_name = id_str.str();
	geometry_msgs::Pose pose;
	pose.position.x = x;
	pose.position.y = y;
	srv.request.initial_pose = pose;
	add_robot.call(srv);

	msg.model_name = id_str.str();
	msg.pose.position.x = pose.position.x;
	msg.pose.position.y = pose.position.y;

	angle = 0;
}

/*void Bot::setColor(float r, float g, float b, float a)
{
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;

	repaint();
}*/

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
	msg.pose.position.x = x;
	msg.pose.position.y = y;

	gazeboPublisher.publish(msg);
}

void Bot::setRPY(geometry_msgs::Pose & p, float roll, float pitch, float yaw)
{
	double halfYaw = double(yaw) * double(0.5);
	double halfPitch = double(pitch) * double(0.5);
	double halfRoll = double(roll) * double(0.5);
	double cosYaw = cos(halfYaw);
	double sinYaw = sin(halfYaw);
	double cosPitch = cos(halfPitch);
	double sinPitch = sin(halfPitch);
	double cosRoll = cos(halfRoll);
	double sinRoll = sin(halfRoll);

	p.orientation.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
	p.orientation.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
	p.orientation.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
	p.orientation.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
}

double Bot::getAngle(tf::Quaternion & q)
{
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	return yaw;
}

void Bot::goTo(float x, float y, float angle)
{
	this->angle = angle;
	float a = fabs(x);
	float b = fabs(y);
	float alpha = atan(a / b);
	alpha = atan(b / a);

	while (alpha > 0.05)
	{
		float da = 0.01;
		angle += da;
		alpha -= da;

		setRPY(msg.pose, 0, 0, angle);
		gazeboPublisher.publish(msg);

		rate.sleep();
	}

	float dx = fabs(this->x - x) / factor;
	dx *= this->x < x ? 1 : -1;

	float dy = fabs(this->y - y) / factor;
	dy *= this->y < y ? 1 : -1;

	for (float step = 0; step < factor && ok(); step++)
	{
		  float ddx = fabs(this->x - x);
		  float ddy = fabs(this->y - y);

		   if(ddx < 1 && ddy < 1) break;

		this->x += dx;
		this->y += dy;

		repaint();

		rate.sleep();
	publishPose();
	}

}