#include <ros/ros.h>
#include <visualization_msgs/Marker.h>  
#include "geometry_msgs/Point.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <cstring>

using namespace std;

float px, py;
string name ;
bool found= false; 

void sendXYToTF(float x, float y, string coord_center, string node_name){
  //coord_center= "world"
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(x, y, 0.0) );
  //tf::Quaternion q;
  //q.setRPY(0, 0, msg->theta);
  transform.setRotation( tf::Quaternion(0, 0, 0, 1) ) ;
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), coord_center, node_name));
	ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "sendXYtoTF -> px: "<< px <<" py: "<< py);
}


tf::StampedTransform listenTF(string coord_center, string node_name)
{
	static   tf::TransformListener listener;
	tf::StampedTransform transform;
    try{
		ros::Time now = ros::Time::now();	
	  listener.waitForTransform(coord_center, node_name, now, ros::Duration(3.0) );
      listener.lookupTransform(coord_center, node_name, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
	  ROS_INFO_STREAM("--Catch Error--");
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      //continue;
    }
	
	
	return transform;
}

 void renderPoint (ros::Publisher & publi, float x, float y)
 {
	 	visualization_msgs::Marker msg;
		msg.header.frame_id = "/rescue";
		msg.header.stamp = ros::Time::now();
		msg.ns = "lost_ns";
		msg.action = visualization_msgs::Marker::ADD;
		msg.pose.position.x = 0;
		msg.pose.position.z = 0;
		msg.pose.orientation.x = 0;
		msg.pose.orientation.z = 1;
		msg.id = 0;
		msg.type = visualization_msgs::Marker::POINTS;
		msg.scale.x = 0.4;
		msg.scale.y = 0.4;
		msg.color.r = 1.0;
		msg.color.g = 0.0;
		msg.color.b = 0.0;
		msg.color.a = 1.0;
			
		geometry_msgs::Point p;
		p.x = x;
		p.y = y;
		p.z = 0;
		msg.points.push_back(p);			
      publi.publish(msg);
 }

void setFind(const std_msgs::String & msg)
{
	std::string data = msg.data;
	if(data.compare("back")==0) found =true;
		
}

int main(int argc, char **argv) {
	ros::init(argc,argv,"lost");
	
	if (argc != 2){ROS_ERROR("need name as argument"); return -1;}
    name = argv[1];
	ROS_INFO_STREAM("name: "<< name);
	
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("rescue",100);
	ros::Subscriber back = nh.subscribe("back",10,setFind);
	
	srand(time(NULL));
	px= rand()%40-20;
	py= rand()%40-20;
	renderPoint(pub,px,py);

	
	ros::Rate rate(30);
	while(ros::ok())
	{
		sendXYToTF(px, py, "world", name);
		renderPoint(pub,px,py);
		
		
		
		if(found && ((px+0.2)!=0.0 || (py+0.2)!=0.0 ) )
		{
			
			tf::StampedTransform transf = listenTF("/world", "/rescuer" );
			px=transf.getOrigin().x()-0.2;
			py=transf.getOrigin().y()-0.2;
			renderPoint(pub,px,py);
			ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "found -- px: "<< px <<" py: "<< py);
		}
		
		ros::spinOnce();
		rate.sleep();
	}
	
	
	return 0;
}