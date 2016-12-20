#include "robot.h"



Robot::Robot(std::string name, int id, float r, float g, float b, float startX, float startY)
{	
	ros::service::waitForService("gazebo/spawn_sdf_model");
	ros::ServiceClient add_robot = 
             m_nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;
 
    std::ifstream fin("/home/lizik/.gazebo/models/pioneer2dx/model.sdf");
 
    std::string model;
    std::string buf;
    while(!fin.eof()){
        getline(fin, buf);
        model += buf + "\n";
    }
    srv.request.model_xml = model;
	srv.request.model_name = name;

	geometry_msgs::Pose pose;
	pose.position.x = startX;
	pose.position.y = startY;
	m_msg.pose = pose;
	m_msg.model_name = name;
	srv.request.initial_pose = pose;
	add_robot.call(srv);


	m_pub = m_nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);

	m_name = name;
	ROS_INFO("Robot %s start.", m_name);
	
	m_anotherX = 0.0;
	m_anotherY = 0.0;
}

void Robot::move(float x, float y)
{
	static ros::Rate loop(30);
	
	double angle = atan2((y - m_msg.pose.position.y), (x - m_msg.pose.position.x));

	tf::Quaternion q(tf::Vector3(0, 0, 1), angle);
	geometry_msgs::Quaternion odom_quat;
  	tf::quaternionTFToMsg(q, odom_quat);
	m_msg.pose.orientation = odom_quat;
	
	float gipo = sqrt(pow((x - m_msg.pose.position.x), 2.0) + 
			pow((y - m_msg.pose.position.y), 2.0));
	unsigned int steps;
	float delta;
	steps = gipo / steplength + 0.5; //count of steps
	delta = gipo / steps; //length of step

	

	float dx = delta * cos(angle);
	float dy = delta * sin(angle);

	for (unsigned int i = 0; i < steps ; i++)
	{
		m_msg.pose.position.x += dx;
		m_msg.pose.position.y += dy;
		m_pub.publish(m_msg);	

		ros::spinOnce();
		loop.sleep();

		sendTransform(m_msg.pose.position.x, m_msg.pose.position.y);
	}
}

bool Robot::isCloseEnough (std::string name)
{
	static tf::TransformListener listener;
	static tf::StampedTransform tr;
	try
	{
		listener.lookupTransform(m_name, name.c_str(), ros::Time(0), tr);

		m_anotherX = tr.getOrigin().x();
		m_anotherY = tr.getOrigin().y();

		if (fabs(m_msg.pose.position.x - m_anotherX) < min_dist &&
			fabs(m_msg.pose.position.y - m_anotherY) < min_dist)
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