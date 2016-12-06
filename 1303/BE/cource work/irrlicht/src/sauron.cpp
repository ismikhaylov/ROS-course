#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <iostream>

using namespace std;
using namespace ros;
using namespace geometry_msgs;

int main(int argc, char ** argv)
{
	init(argc, argv, "sauron");

	NodeHandle n;
	Pose2D current_pose, next_pose;
	Publisher pubSauron = n.advertise<Pose2D>("/sauron/pose", 10);

	Rate loop_rate(10);

	current_pose.x = 0;
	current_pose.y = 0;

	while (n.ok())
	{
		next_pose.x = 1 + rand() % 17;
		next_pose.y = 1 + rand() % 17;

		float factor = 30;

		float dx = fabs(next_pose.x - current_pose.x) / factor;
		dx *= next_pose.x > current_pose.x ? 1 : -1;
		float dy = fabs(next_pose.y - current_pose.y) / factor;
		dy *= next_pose.y > current_pose.y ? 1 : -1;

		for (float step = 0; step < factor; step++)
		{
			current_pose.x += dx;
			current_pose.y += dy;

			pubSauron.publish(current_pose);

			loop_rate.sleep();
		}

		spinOnce();
	}

	return 0;
}