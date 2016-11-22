#include "tf_lab/bot.h"
#include <geometry_msgs/Point.h>

using namespace tf;

int Bot::count = 0;

Bot::Bot(){
	Bot("partner_bot", "bot", "bot_marker", 0, 0);
}

Bot::Bot(string partner_name, string name, string rviz_marker_topic, float x, float y){
	count++;
	id = count;

	this->partner_name = partner_name;
	delta = 0.1;

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

	partner_pos.setOrigin(Vector3(x - 10, y - 10, 0));

	pub = node.advertise<visualization_msgs::Marker>(rviz_marker_topic,10,true);

	move(x, y);
}

void Bot::repaint(){
	pub.publish(marker);
}

void Bot::broadcastPose(){
	Quaternion q;
	q.setRPY(0, 0, 0);

	Transform transform;
	transform.setOrigin(Vector3(marker.pose.position.x, marker.pose.position.y, 0.0));
	transform.setRotation(q);

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), base_frame_id, child_frame_id));
}

void Bot::updatePartnerPos(){
	//listener.waitForTransform(base_frame_id, partner_name, Time::now(), Duration(0.1));
	try{
      	listener.lookupTransform(base_frame_id, partner_name,
                               ros::Time(0), partner_pos);
    }
    catch (TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
}

bool Bot::isMet(){
	updatePartnerPos();

	float partner_x = partner_pos.getOrigin().x();
	float partner_y = partner_pos.getOrigin().y();

	float my_x = getX();
	float my_y = getY();

	if( (pow(partner_x - my_x,2) + pow(partner_y - my_y, 2)) <= pow(delta,2) ){
		return true;
	} else{
		return false;
	}

}


void Bot::move(float x, float y){
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
