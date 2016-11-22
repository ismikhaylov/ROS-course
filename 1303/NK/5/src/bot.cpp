#include "gazebo_lab/bot.h"
#include "gazebo_msgs/SpawnModel.h"
#include <fstream> 
#include "stdlib.h"

Bot::Bot(){
	Bot("bot", "partner_bot", 0, 0, 0);
}

Bot::Bot(string my_name, string partner_name, float x, float y, float w){
    this->x = x;
    this->y = y;
    this->w = w;
    this->my_name = my_name;
    this->partner_name = partner_name;
    isPartnerOnMap = false;
    delta = 0.7;

	ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot = node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;
 
    ifstream fin("/home/ckyhc/.gazebo/models/pioneer2dx/model.sdf");
    
    string model;
    string buf;
    while(!fin.eof()){
        getline(fin, buf);
        model += buf + "\n";
    }
    srv.request.model_xml = model;
    srv.request.model_name = this->my_name;


    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    srv.request.initial_pose = pose;
    add_robot.call(srv);
    pub = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    broadcastPose();
}

void Bot::broadcastPose(){
	Quaternion q;
	q.setRPY(0, 0, 0);

	Transform transform;
	transform.setOrigin(Vector3(x, y, 0.0));
	transform.setRotation(q);

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", my_name));
}

void Bot::updatePartnerPos(){
	float p_x = partner_pos.getOrigin().x();
	float p_y = partner_pos.getOrigin().y();
	isPartnerOnMap = true;
	try{
      	listener.lookupTransform(my_name, partner_name,
                               ros::Time(0), partner_pos);
    }
    catch (TransformException &ex) {
		isPartnerOnMap = false;
      	ROS_ERROR("%s",ex.what());
      	ros::Duration(1.0).sleep();
    }
}

bool Bot::isMet(){
	updatePartnerPos();

	float p_x = partner_pos.getOrigin().x();
	float p_y = partner_pos.getOrigin().y();

	if((pow(p_x,2) + pow(p_y, 2)) <= pow(delta,2)){
		return true && isPartnerOnMap;
	} else{
		return false && isPartnerOnMap;
	}

}

void Bot::updatePose(){
	gazebo_msgs::ModelState msg;
    msg.model_name = my_name;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.orientation.z = 1;
    msg.pose.orientation.w = w;

    pub.publish(msg);
	broadcastPose();
}