#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

bool fl;

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "C3Po");

	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::Publisher pub = n.advertise<std_msgs::String>("R2D2/cmd", 10);

	ros::Rate loop_rate(100);

	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::TransformListener listener;
	tf::StampedTransform tr;

	visualization_msgs::Marker marker;
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();

	marker.ns = "basic_shapes";
	marker.id = 0;

	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = 17;
	marker.pose.position.y = 17;
	marker.pose.position.z = 0;

	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;

	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);

	float d = 0.1;
	float start_x = marker.pose.position.x;
	float start_y = marker.pose.position.y;
	fl = false;

	while (ros::ok())
	{

		// Waiting until R2D2 is lost.
		if (pub.getNumSubscribers() == 0)
		{
			sleep(1);
			marker_pub.publish(marker);
			continue;
		}

		if (!fl)
		{
			try
			{
				listener.lookupTransform("/world", "/R2D2/pose", ros::Time(0), tr);
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s", ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}
			if (fabs(marker.pose.position.x - tr.getOrigin().x() < 0.05) && fabs(marker.pose.position.y - tr.getOrigin().y()) < 0.05)
			{
				std_msgs::String msg;
				msg.data = "!";
				fl = true;

				pub.publish(msg);
				continue;
			}

			float factor = 100;

			float dx = fabs(marker.pose.position.x - tr.getOrigin().x()) / factor;
			dx *= marker.pose.position.x < tr.getOrigin().x() ? 1 : -1;
			float dy = fabs(marker.pose.position.y - tr.getOrigin().y()) / factor;
			dy *= marker.pose.position.y < tr.getOrigin().y() ? 1 : -1;

			marker.pose.position.x += dx;
			marker.pose.position.y += dy;

			marker_pub.publish(marker);

			loop_rate.sleep();

		}
		else
		{
			float factor = 100;

			float dx = fabs(marker.pose.position.x - start_x) / factor;
			dx *= marker.pose.position.x < start_x ? 1 : -1;
			float dy = fabs(marker.pose.position.y - start_y) / factor;
			dy *= marker.pose.position.y < start_y ? 1 : -1;

			for (float step = 0; step < factor; step++)
			{
				marker.pose.position.x += dx;
				marker.pose.position.y += dy;

				marker_pub.publish(marker);

				transform.setOrigin( tf::Vector3(marker.pose.position.x, marker.pose.position.y, 0.0) );
				tf::Quaternion q;
				q.setRPY(0, 0, 0);
				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "C3Po/pose"));

				loop_rate.sleep();

			}

			return 0;

		}

		ros::spinOnce();
	}

	return 0;
}


