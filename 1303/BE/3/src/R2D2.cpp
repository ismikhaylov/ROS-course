#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

bool fl;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	fl = true;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "R2D2");

	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::Subscriber sub = n.subscribe("R2D2/cmd", 10, chatterCallback);

	ros::Rate loop_rate(100);

	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::TransformListener listener;
	tf::StampedTransform tr;

	visualization_msgs::Marker marker;
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();

	marker.ns = "basic_shapes";
	marker.id = 1;

	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
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

	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);

	float d = 0.1;
	fl = false;
	int i = 0;

	while (ros::ok())
	{

		if (fl)
		{
			try
			{
				listener.lookupTransform("/world", "/C3Po/pose", ros::Time(0), tr);
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s", ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}

			float factor = 100;

			float dx = fabs(marker.pose.position.x - tr.getOrigin().x()) / factor;
			dx *= marker.pose.position.x < tr.getOrigin().x() ? 1 : -1;
			float dy = fabs(marker.pose.position.y - tr.getOrigin().y()) / factor;
			dy *= marker.pose.position.y < tr.getOrigin().y() ? 1 : -1;

			if(dx < 0.0005 && dy < 0.0005) return 0;

			marker.pose.position.x += dx;
			marker.pose.position.y += dy;

			marker_pub.publish(marker);
		}
		else
		{
			i++;
			if (i < 10 || true)
			{
				int new_x = rand() % 10;
				int new_y = rand() % 10;
				float factor = 100;

				new_x *= rand() % 2 == 0 ? 1 : -1;
				new_y *= rand() % 2 == 0 ? 1 : -1;

				float dx = fabs(marker.pose.position.x - new_x) / factor;
				dx *= marker.pose.position.x < new_x ? 1 : -1;
				float dy = fabs(marker.pose.position.y - new_y) / factor;
				dy *= marker.pose.position.y < new_y ? 1 : -1;

				for (float step = 0; step < factor && !fl; step++)
				{
					marker.pose.position.x += dx;
					marker.pose.position.y += dy;

					marker_pub.publish(marker);

					transform.setOrigin( tf::Vector3(marker.pose.position.x, marker.pose.position.y, 0.0) );
					tf::Quaternion q;
					q.setRPY(0, 0, 0);
					transform.setRotation(q);
					br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "world", "R2D2/pose"));

					ros::spinOnce();
					loop_rate.sleep();

				}
			}
		}

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}