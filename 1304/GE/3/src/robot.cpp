#include "robot.h"



Robot::Robot(std::string name, int id, float r, float g, float b, float startX, float startY)
{
	m_anotherX = 0.0;
	m_anotherY = 0.0;
	m_name = name;
	m_pub = m_nh.advertise<visualization_msgs::Marker>(name.c_str(), 10, true);
	m_marker.header.frame_id = "/m_frame";
	m_marker.header.stamp = ros::Time::now();
	m_marker.ns = "there_is_point";
	m_marker.action = visualization_msgs::Marker::ADD;
	m_marker.pose.orientation.w = 1;
	m_marker.id = id;
	m_marker.type = visualization_msgs::Marker::SPHERE;
	m_marker.scale.x = 1.0;
	m_marker.scale.y = 1.0;
	m_marker.scale.z = 1.0;
	m_marker.pose.position.x = startX;
	m_marker.pose.position.y = startY;
	m_marker.color.r = r;
	m_marker.color.g = g;
	m_marker.color.b = b;
	m_marker.color.a = 1.0;
	geometry_msgs::Point p;
	p.x = 0;
	p.y = 0;
	p.z = 0;
	m_marker.points.push_back(p);
	m_marker.lifetime = ros::Duration();
	m_pub.publish(m_marker);
}

void Robot::move(float x, float y)
{
	static ros::Rate loop(30);
	
	float gipo = sqrt(pow((x - m_marker.pose.position.x), 2.0) + 
			pow((y - m_marker.pose.position.y), 2.0));
	unsigned int steps;
	float delta;
	steps = gipo / steplength + 0.5; //count of steps
	delta = gipo / steps; //length of step

	double angle = atan2((y - m_marker.pose.position.y), (x - m_marker.pose.position.x));

	float dx = delta * cos(angle);
	float dy = delta * sin(angle);

	for (unsigned int i = 0; i < steps ; i++)
	{
		m_marker.pose.position.x += dx;
		m_marker.pose.position.y += dy;
		m_pub.publish(m_marker);		

		sendTransform(m_marker.pose.position.x, m_marker.pose.position.y);

		ros::spinOnce();
		loop.sleep();
	}
}

bool Robot::isCloseEnough (std::string name)
{
	static tf::TransformListener listener;
	static tf::StampedTransform tr;
	try
	{
		listener.lookupTransform("world", name.c_str(), ros::Time(0), tr);

		m_anotherX = tr.getOrigin().x();
		m_anotherY = tr.getOrigin().y();

		if (fabs(m_marker.pose.position.x - m_anotherX) < min_dist &&
			fabs(m_marker.pose.position.y - m_anotherY) < min_dist)
			return true;
	}	
	catch (tf::TransformException &ex)
	{
		ros::Duration(1.0).sleep();
	}

	return false;
}

void Robot::sendTransform(float x, float y)
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x, y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", m_name));
}