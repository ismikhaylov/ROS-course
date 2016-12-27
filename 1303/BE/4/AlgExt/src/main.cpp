#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sstream>
#include <cmath>
#include <string>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;
using namespace ros;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace sensor_msgs;

/* LET'S USE PROCEDURE ORIENTED STILE TO SIMPLIFY OUR LIFE */
/* THE GOAL IS JUST TO BE SIMPLY AND FASTER AT ALL         */

Pose pose;

float target_x;
float target_y;

bool at_x = false;
bool at_y = false;

float l_scan = 0;
float r_scan = 0;
float c_scan = 0;
float distance_ = 0;

float rotation = 0;

Publisher publisher;

Twist msg;

void LaserScanCallback(const LaserScan::ConstPtr & msg)
{
	l_scan = msg->ranges[msg->ranges.size() - 1];
	c_scan = msg->ranges[msg->ranges.size() / 2];
	r_scan = msg->ranges[0];

	distance_ = min( min(l_scan, r_scan), c_scan);
}

void BasePoseCallback(const Odometry::ConstPtr & msg)
{
	pose = msg->pose.pose;

	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);

	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	
	rotation = yaw;
}

inline void Sleep()
{
	Rate rate(10);
	rate.sleep();
}

bool YNormalized()
{
	float condition = pose.position.y < target_y ? 1.6 : -1.6;
	return fabs( round(rotation * 10) / 10 - condition ) < 0.1;
}

void NormalizeY()
{
	msg.angular.z = 1;
	msg.linear.x = 0;


	while (!YNormalized())
	{
		publisher.publish(msg);
		Sleep();
		spinOnce();
	}

	msg.angular.z = 0;
	msg.linear.x = 1;
}

bool XNormalized()
{
	float condition = pose.position.x < target_x ? 0 : 3;
	return fabs( round(rotation * 10) / 10 - condition ) < 0.1;
}

void NormalizeX()
{
	msg.angular.z = -1;
	msg.linear.x = 0;


	while (!XNormalized())
	{
		publisher.publish(msg);
		Sleep();
		spinOnce();
	}

	msg.angular.z = 0;
	msg.linear.x = 1;
}

void Rotate()
{
	msg.angular.z = 1;
	msg.linear.x = 0;

	while (distance_ < 0.5)
	{
		publisher.publish(msg);
		Sleep();
		spinOnce();
	}

	msg.angular.z = 0;
	msg.linear.x = 1;
}

bool CheckX()
{
	at_x = round(fabs(pose.position.x - target_x) * 10) / 10 == 0.1;
	return at_x;
}

bool CheckY()
{
	at_y = round(fabs(pose.position.y - target_y) * 10) / 10 == 0.1;
	return at_y;
}

void GetRound()
{
	Rotate();

	while (distance_ >= 0.5)
	{
		publisher.publish(msg);
		Sleep();
		spinOnce();
	}
}

void GoAhead()
{
	publisher.publish(msg);
	Sleep();
	spinOnce();
	cout << "pose=(" << pose.position.x << "; " << pose.position.y << ")" << endl;
}

void Stop()
{
	msg.linear.x = 0;
	publisher.publish(msg);
}

int main(int argc, char ** argv)
{
	/* INITIALIZATION */

	stringstream ss;
	for (int i = 1; i < argc; i++)
		ss << argv[i] << ' ';

	ss >> target_x >> target_y;

	init(argc, argv, "AlgExt");

	NodeHandle nodeHandler;
	publisher = nodeHandler.advertise<Twist>("/cmd_vel", 1000);
	Subscriber scanSubscriber = nodeHandler.subscribe("/base_scan", 1000, LaserScanCallback);
	Subscriber basePoseSubscriber = nodeHandler.subscribe("/base_pose_ground_truth", 1000, BasePoseCallback);

	/* WAIT FOR ALL PUBLISHERS */

	while (scanSubscriber.getNumPublishers() == 0 || basePoseSubscriber.getNumPublishers() == 0)
		sleep(1);

	/* GO AHEAD */

	while (ok())
	{
		NormalizeX();

		while (ok() && !at_x)
		{
			if (distance_ < 0.5)
			{
				at_x = at_y = false;
				GetRound();
				break;
			}

			CheckX();

			if (!at_x)
				GoAhead();
			else
				break;
		}

		if (!at_x) 
			continue;

		NormalizeY();

		while (ok() && !at_y)
		{
			if (distance_ < 0.5)
			{
				at_x = at_y = false;
				GetRound();
				break;
			}

			CheckY();

			if (!at_y)
				GoAhead();
			else
				break;
		}

		if (!at_y) 
			continue;

		if (at_x && at_y)
		{
			cout << "CONGRATULATIONS!! MISSION COMPLETED" << endl;
			shutdown();
			return 0;
		}
	}

	return 0;
}