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

float px=0.0, py=0.0;
string name ;
float lost_x, lost_y;
bool hasFound=false;

//std::string turtle_name==node_name;

void sendXYToTF(float x, float y, string coord_center, string node_name){
  //coord_center= "world"
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(x, y, 0.0) );
  //tf::Quaternion q;
  //q.setRPY(0, 0, msg->theta);
  transform.setRotation( tf::Quaternion(0, 0, 0, 1) ) ;
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), coord_center, node_name));
	ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "sendXYtoTF --> px: "<< px <<" py: "<< py);
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
		msg.ns = "rescuer_ns";
		msg.action = visualization_msgs::Marker::ADD;
		msg.pose.position.x = 0;
		msg.pose.position.z = 0;
		msg.pose.orientation.x = 0;
		msg.pose.orientation.z = 1;
		msg.id = 1;
		msg.type = visualization_msgs::Marker::POINTS;
		msg.scale.x = 0.4;
		msg.scale.y = 0.4;
		msg.color.r = 0.5;
		msg.color.g = 1.0;
		msg.color.b = 1.0;
		msg.color.a = 1.0;
			
		geometry_msgs::Point p;
		p.x = x;
		p.y = y;
		p.z = 0;
		msg.points.push_back(p);
			
    publi.publish(msg);
 }




int main(int argc, char ** argv)
{
	ros::init(argc, argv, "rescuer");
   
	if (argc != 2){ROS_ERROR("need name as argument"); return -1;};
    name = argv[1];
	ROS_INFO_STREAM("name: "<< name);
	 
	ros::NodeHandle nh;
	ros::Publisher pub =  nh.advertise<visualization_msgs::Marker>("rescue",100);
	ros::Publisher back= nh.advertise<std_msgs::String>("back",10);
	std_msgs::String Message;
    Message.data = std::string("back");
	renderPoint(pub,px,py);

	
	ros::Rate rate(300);
	while(ros::ok())
	{
	  
	     if(!hasFound)
		 {
        	tf::StampedTransform transf = listenTF("/world", "/lost" );
	   		lost_x=transf.getOrigin().x();
			lost_y=transf.getOrigin().y();
			float dx = fabsf(lost_x - px)==0.0 ? 0.0 :(lost_x - px)/ fabsf(lost_x - px)/10;
			float dy = fabsf(lost_y - py)==0.0 ? 0.0 :(lost_y - py)/ fabsf(lost_y - py)/10;
			px+=dx;
			py+=dy;
			renderPoint(pub,px,py);
			ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "Not hasFound -- px: "<< px <<" py: "<< py);
		}
		
		if(hasFound && (px!=0.0 || py!=0.0 ) )
		{
			float dx = fabsf(0 - px)==0.0 ? 0.0 :(0.0 - px)/ fabsf(0.0 - px)/10;
			float dy = fabsf(0 - py)==0.0 ? 0.0 :(0.0 - py)/ fabsf(0.0 - py)/10;
			px+=dx;
			py+=dy;
			sendXYToTF(px, py, "world", name);
			renderPoint(pub,px,py);
			ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "hasFound -- px: "<< px <<" py: "<< py);
			tf::StampedTransform transf = listenTF("/world", "/lost" );
		}
		
		if( fabs(px - lost_x)<0.2 && fabsf(py - lost_y)<0.2 )
		{
			back.publish(Message);
			hasFound= true;
		}
		
	   ros::spinOnce();	
	   rate.sleep();
    }


	return 0;
}