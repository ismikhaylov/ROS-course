#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

bool fl;

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "one");

	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::Publisher pub = n.advertise<std_msgs::String>("one", 10);

	ros::Rate loop_rate(50);

	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::TransformListener listener;
	tf::StampedTransform tr;

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();

	marker.ns = "basic_shapes";
	marker.id = 0;

	// Set our initial shape type to be a cube
  	uint32_t shape = visualization_msgs::Marker::CUBE;
	shape = visualization_msgs::Marker::CYLINDER;
	shape = visualization_msgs::Marker::ARROW;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = 10;
	marker.pose.position.y = 10;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	
	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);

	float start_x = marker.pose.position.x;
	float start_y = marker.pose.position.y;
	fl = false;

	while (ros::ok())
	{
		// Sleep for moving
		if (pub.getNumSubscribers() == 0)
		{
			continue;
		}

		if (!fl)
		{
			listener.lookupTransform("/world", "/one/pose", ros::Time(0), tr);
			
			// Boundary contact
			if (fabs(marker.pose.position.x - tr.getOrigin().x() < 0.5) && fabs(marker.pose.position.y - tr.getOrigin().y()) < 0.5)
			{
				std_msgs::String msg;
				msg.data = "!";
				fl = true;
				pub.publish(msg);
				continue;
			}

			// Steeps for search
			float dx = fabs(marker.pose.position.x - tr.getOrigin().x()) / 100;
			dx *= marker.pose.position.x < tr.getOrigin().x() ? 1 : -1;
			float dy = fabs(marker.pose.position.y - tr.getOrigin().y()) / 100;
			dy *= marker.pose.position.y < tr.getOrigin().y() ? 1 : -1;

			marker.pose.position.x += dx;
			marker.pose.position.y += dy;

			marker_pub.publish(marker);

			loop_rate.sleep();
		}
		else
		{
			// Steps after search
			float dx = fabs(marker.pose.position.x - start_x) / 100;
			dx *= marker.pose.position.x < start_x ? 1 : -1;
			float dy = fabs(marker.pose.position.y - start_y) / 100;
			dy *= marker.pose.position.y < start_y ? 1 : -1;

			while(marker.pose.position.x <= start_x && marker.pose.position.y <= start_y)
			{
				marker.pose.position.x += dx;
				marker.pose.position.y += dy;

				marker_pub.publish(marker);

				transform.setOrigin( tf::Vector3(marker.pose.position.x, marker.pose.position.y, 0.0) );
				tf::Quaternion q;
				q.setRPY(0, 0, 0);
				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "two/pose"));
				loop_rate.sleep();
			}

			return 0;
		}
	}

	return 0;
}


