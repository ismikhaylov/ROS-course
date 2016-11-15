#include "robot.h"

Robot::Robot(const char* name)
{
	ros::service::waitForService("gazebo/spawn_sdf_model");
	ros::ServiceClient add_robot = 
             _nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;
 
    ifstream fin("/home/sergei/.gazebo/models/pioneer2dx/model.sdf");
 
    string model;
    string buf;
    while(!fin.eof()){
        getline(fin, buf);
        model += buf + "\n";
    }
    srv.request.model_xml = model;
    srv.request.model_name = name;
    geometry_msgs::Pose pose;
    pose.position.x = (rand() % 20) - 10;
    pose.position.y = (rand() % 20) - 10;
    pose.orientation.z = 30;
    _msg.pose = pose;
    _msg.model_name = name;
    srv.request.initial_pose = pose;
    add_robot.call(srv);

    _pub = _nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);

	_name = name;
	ROS_INFO( "Robot %s start.", _name );

	
	_aRX = _aRY = 0.0;
}


void Robot::move(float x, float y)
{	
	static tf::TransformBroadcaster br;
	static tf::Transform transform;

	float angle = atan( (_msg.pose.position.y - y) / (_msg.pose.position.x - x) );
	
	if (_msg.pose.position.x < x) {
		angle += M_PI;
	}
	tf::Quaternion q(tf::Vector3(0, 0, 1), angle);
	geometry_msgs::Quaternion odom_quat;
  	tf::quaternionTFToMsg(q, odom_quat);
	_msg.pose.orientation = odom_quat;

	float dx = fabs(_msg.pose.position.x - x) / STEPS;
	float dy = fabs(_msg.pose.position.y - y) / STEPS;
	dx *= _msg.pose.position.x < x ? 1 : -1;
	dy *= _msg.pose.position.y < y ? 1 : -1;

	ros::Rate rate(50);

	for (int step = 0; step < STEPS && ros::ok(); step++) {
		
		_msg.pose.position.x += dx;
		_msg.pose.position.y += dy;
		_pub.publish( _msg );

		ros::spinOnce();
		rate.sleep();

		transform.setOrigin( tf::Vector3(_msg.pose.position.x, _msg.pose.position.y, 0.0) );	
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", _name));
	}
	ROS_INFO( "Position: %f %f", _msg.pose.position.x, _msg.pose.position.y );
}

void Robot::move(float x, float y, float distance) 
{
	float dx = fabs(_msg.pose.position.x - x);
	float dy = fabs(_msg.pose.position.y - y);

	dx = (dx < distance ? dx : distance ) * ( _msg.pose.position.x < x ? 1 : - 1 );
	dy = (dy < distance ? dy : distance ) * ( _msg.pose.position.y < y ? 1 : - 1 );

	move( _msg.pose.position.x + dx, _msg.pose.position.y + dy );
}


bool Robot::checkPosAnotherRobot(const char* tf)
{
	static tf::TransformListener listener;
	static tf::StampedTransform tr;
	try
	{
		listener.lookupTransform(_name, tf, ros::Time(0), tr);
		
		_aRX = tr.getOrigin().x() + _msg.pose.position.x;
		_aRY = tr.getOrigin().y() + _msg.pose.position.y;

		ROS_INFO( "Position Another: %f %f", _aRX, _aRY );

		if (fabs(_msg.pose.position.x - _aRX) < R 
				&& fabs(_msg.pose.position.y - _aRY) < R) {
			return true;
		}
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s", ex.what());
	}
	return false;
}

void Robot::moveToAnotherRobot(float distance)
{
	if (distance) {
		move( _aRX, _aRY, distance);
	} else {		
		move( _aRX, _aRY );
	}
}


bool Robot::isPosition(float x, float y) 
{
	if (fabs(_msg.pose.position.x-x) < DXY
			&& fabs(_msg.pose.position.y-y) < DXY) {
		return true;
	}
	return false;
}