#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

bool fl;
int chet = 1; 

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	fl = true;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "two");
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::Subscriber sub = n.subscribe("one", 10, chatterCallback);

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
	marker.id = 1;

	// Set our initial shape type to be a cube
  	uint32_t shape = visualization_msgs::Marker::CUBE;
	shape = visualization_msgs::Marker::CYLINDER;
	shape = visualization_msgs::Marker::ARROW;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
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
	marker.color.r = 1.0f;
	marker.color.g = 1.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);

	fl = false;
	int j = 1;
	

	while (ros::ok())
	{
		j ++;
		if (fl)
		{
			// Steps after contact
			listener.lookupTransform("/world", "/two/pose", ros::Time(0), tr);
			float dx = fabs(marker.pose.position.x - tr.getOrigin().x());
			dx *= marker.pose.position.x < tr.getOrigin().x() ? 1 : -1;
			float dy = fabs(marker.pose.position.y - tr.getOrigin().y());
			dy *= marker.pose.position.y < tr.getOrigin().y() ? 1 : -1;

			if(dx < 0.00001 && dy < 0.00001) return 0;

			marker.pose.position.x += dx;
			marker.pose.position.y += dy;

			marker_pub.publish(marker);
		}
		else
		{
			// Steps before contact
			float new_x = (j);
			float new_y = (j);
			
			float dx = (j)/2;
			float dy = (j)/2;

			if(chet == 1){
				dx *= 1;
				dy *= 1;	
				chet = 2;
			}
			else if(chet == 2){
				dx *= -1;
				dy *= 1;		
				chet = 3;
			}
			else if(chet == 3){
				dx *= -1;
				dy *= -1;		
				chet = 4;
			}
			else if(chet == 4){
				dx *= 1;
				dy *= -1;		
				chet = 1;
			}

			for(int f = 0; f < 10; f++){
				marker.pose.position.x += dx/(100);
				marker.pose.position.y += dy/(100);

				marker_pub.publish(marker);

				transform.setOrigin( tf::Vector3(marker.pose.position.x, marker.pose.position.y, 0.0) );
				tf::Quaternion q;
				q.setRPY(0, 0, 0);
				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "world", "one/pose"));
				loop_rate.sleep();
			}
			
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
