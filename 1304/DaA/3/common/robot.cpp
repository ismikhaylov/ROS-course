#include "robot.h"
#include <geometry_msgs/Point.h>

using namespace tf;


Vector3 listen_position(string frame_id, string target, TransformListener listener) {
	try {
		StampedTransform target_pos;
      	listener.lookupTransform(frame_id, target, ros::Time(0), target_pos);
      	return target_pos.getOrigin();
    } catch (TransformException &ex) {
    	ROS_ERROR("%s",ex.what());
      	ros::Duration(1.0).sleep();
      	return Vector3();
    }
}

bool move(Bot *bot, float target_x, float target_y) {
	float bot_x = bot->getX();
	float bot_y = bot->getY();

	float l = sqrt(pow(target_x-bot_x, 2) + pow(target_y-bot_y, 2));
	float delta = 2.0;
	if (l == delta) {
		return true;
	} else if (l < delta) {
		bot->move(target_x, target_y);
		return true;
	}
	
	float new_x = bot_x + delta*abs(target_x-bot_x)/l;
	float new_y = bot_y + delta*abs(target_y-bot_y)/l;

	bot->move(new_x, new_y);
	return false;
}

bool collision(float x1, float y1, float x2, float y2, float delta) {
	return (pow(x1 - x2, 2) + pow(y1 - y2, 2)) <= pow(delta, 2);
}

Bot::Bot(string name, string rviz_marker_topic, float x, float y){
	count++;
	id = count;
	speed = 0.1;
	rviz_frame_id = "/lab3";
	rviz_namespace = "lab3";
	base_frame_id = "world";
	child_frame_id = name;
	this->rviz_marker_topic = rviz_marker_topic;
	marker.header.frame_id = rviz_frame_id;
	marker.header.stamp = ros::Time::now();
	marker.ns = rviz_namespace;
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.id = id;
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	marker.color.a = 1.0;

	geometry_msgs::Point p;
	marker.points.push_back(p);

	pub = node.advertise<visualization_msgs::Marker>(rviz_marker_topic,10,true);

	move(x, y);
}

void Bot::draw() {
	pub.publish(marker);
}

void Bot::broadcast() {
	Quaternion q;
	q.setRPY(0, 0, 0);

	Transform transform;
	transform.setOrigin(Vector3(marker.pose.position.x, marker.pose.position.y, 0.0));
	transform.setRotation(q);

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_frame_id, child_frame_id));
}

void Bot::move(float x, float y) {
	marker.pose.position.x = x;
	marker.pose.position.y = y;

	repaint();
	broadcastPose();
}

void Bot::setColor(float r, float g, float b){
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
}

float Bot::getX(){
	return marker.pose.position.x;
}

float Bot::getY(){
	return marker.pose.position.y;
}