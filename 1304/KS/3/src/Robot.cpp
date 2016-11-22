#include "Robot.h"

Robot::Robot(const char* name, const int id)
{
	_name = name;
	ROS_INFO( "Robot %s start.", _name );
	_markerPub = _nh.advertise<visualization_msgs::Marker>("mView", 1);
	
	_marker.header.frame_id = "/my_frame";
	_marker.header.stamp = ros::Time::now();

	_marker.ns = "basic_shapes";
	_marker.id = id;

	_marker.type = visualization_msgs::Marker::SPHERE;
	_marker.action = visualization_msgs::Marker::ADD;

	_marker.pose.position.x = 0;
	_marker.pose.position.y = 0;
	_marker.pose.position.z = 0;

	_marker.pose.orientation.x = 0.0;
	_marker.pose.orientation.y = 0.0;
	_marker.pose.orientation.z = 0.0;
	_marker.pose.orientation.w = 1.0;

	_marker.scale.x = 1.0;
	_marker.scale.y = 1.0;
	_marker.scale.z = 1.0;

	_marker.color.r = 1.0f;
	_marker.color.g = 0.0f;
	_marker.color.b = 0.0f;
	_marker.color.a = 1.0;

	_marker.lifetime = ros::Duration();
	_markerPub.publish(_marker);
	_aRX = _aRY = 0.0;
}


void Robot::move(double x, double y)
{	
	static tf::TransformBroadcaster br;
	static tf::Transform transform;
	static ros::Rate loop(30);

	float dx = fabs(_marker.pose.position.x - x) / STEPS;
	float dy = fabs(_marker.pose.position.y - y) / STEPS;
	dx *= _marker.pose.position.x < x ? 1 : -1;
	dy *= _marker.pose.position.y < y ? 1 : -1;

	for (float step = 0; step < STEPS ; step++)
	{
		_marker.pose.position.x += dx;
		_marker.pose.position.y += dy;
		_markerPub.publish(_marker);		

		transform.setOrigin( tf::Vector3(_marker.pose.position.x, _marker.pose.position.y, 0.0) );	
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", _name));

		ros::spinOnce();
		loop.sleep();
	}

	ROS_INFO( "Position: %f %f", _marker.pose.position.x, _marker.pose.position.y );
}

bool Robot::checkPosAnotherRobot(const char* tf)
{
	static tf::TransformListener listener;
	static tf::StampedTransform tr;
	try
	{
		listener.lookupTransform("/world", tf, ros::Time(0), tr);
		
		_aRX = tr.getOrigin().x();
		_aRY = tr.getOrigin().y();

		ROS_INFO( "Position Another: %f %f", _aRX, _aRY );

		if (fabs(_marker.pose.position.x - _aRX) < R 
				&& fabs(_marker.pose.position.y - _aRY) < R) {
			return true;
		}
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}
	return false;
}

void Robot::moveToAnotherRobot()
{
	move( _aRX, _aRY );
}
