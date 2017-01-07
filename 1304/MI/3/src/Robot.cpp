#include "Robot.h"

Robot::Robot(ros::NodeHandle &robotNode, string name) : robotNode(robotNode), name(name) {}

void Robot::initVisualisation() {
	marker_pub = robotNode.advertise<visualization_msgs::Marker>("visualization_marker", 100);
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();
	marker.lifetime = ros::Duration();
	marker.ns = name + "_namespace";
	marker.action = visualization_msgs::Marker::ADD;
	marker.id = id;

	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 0.0;

	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
		
	marker.pose.position.x = 0.0;
	marker.pose.position.y = 0.0;
	marker.pose.position.z = 0.0;

	marker.color.a = 1.0;
}

void Robot::sendPos(string typeRobot) {
	static tf::TransformBroadcaster br;
	tf::Transform transform;

	cout << typeRobot << " moves to x = " << x << "; y =" << y << endl;

	transform.setOrigin(tf::Vector3(x, y, 0.0));
	tf::Quaternion q;
	q.setRPY(0, 0, z);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
}	

void Robot::updateMarker() {
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = z;
	marker_pub.publish(marker);
}	

void Robot::set_shape(string currState)
{
		//lost
	if(currState == "lost")
	{
		marker.type = visualization_msgs::Marker::CUBE;
		marker.color.r = 1.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;
	}
	//follow
	if(currState == "follow")
	{
		marker.type = visualization_msgs::Marker::CUBE;
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
	}
	//home
	if(currState == "home")
	{
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.color.r = 1.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
	}

	if(currState == "search")
	{
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.color.r = 0.0f;
		marker.color.g = 0.0f;
		marker.color.b = 1.0f;
	}
	if(currState == "leads")
	{
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 1.0f;
	}
}