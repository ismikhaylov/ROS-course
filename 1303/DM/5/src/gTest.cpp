#include <ros/ros.h>
#include <visualization_msgs/Marker.h>  
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <cstring>

#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/GetModelState.h"
#include <fstream>

using namespace std;

float px, py;
float dx, dy;
string name ;
bool found= false; 
ros::ServiceClient srv_client ;
float test_angle, alpha;
bool success_listen;


gazebo_msgs::GetModelState getSate(ros::NodeHandle & nodeh , std::string relativeEntityName , std::string modelName )
{
  gazebo_msgs::GetModelState getModelState ;
	
  srv_client = nodeh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state") ;
  getModelState.request.model_name = modelName ;
  getModelState.request.relative_entity_name = relativeEntityName ;
  srv_client.call(getModelState) ;
	
	return getModelState;
}


void sendXYToTF( ros::NodeHandle & nodeh, std::string relativeEntityName , std::string modelName )
{
	
  static tf::TransformBroadcaster br;
  tf::Transform transform;
 
  //std::string modelName = (std::string)"gRescuer" ;
  //std::string relativeEntityName = (std::string)"world" ;

  gazebo_msgs::GetModelState getModelState ;
  geometry_msgs::Point pp ;
  geometry_msgs::Quaternion qq ;
	
  getModelState= getSate(nodeh , relativeEntityName , modelName );

  pp = getModelState.response.pose.position ;
  qq = getModelState.response.pose.orientation ;
        
  transform.setOrigin( tf::Vector3(pp.x, pp.y, pp.z) ) ;
  tf::Quaternion q(qq.x, qq.y, qq.z, qq.w) ;
  transform.setRotation(q) ;
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), relativeEntityName, modelName)) ;
	
  //perform orientation angle	
  double roll, pitch, yaw;	
  tf::Matrix3x3 matr(q);
  matr.getRPY(roll, pitch, yaw);
  test_angle=yaw;
	ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << " test_angle: "<<test_angle);
  if (test_angle<0){
	  test_angle+= (2*M_PI);
	  if (fabs(test_angle-2*M_PI)<0.1 ) test_angle =0.0;
  }
	else{
		if (fabs(test_angle-2*M_PI)<0.1 ) test_angle =0.0;
	}
	
  //perfom x, y
  px=pp.x;
  py=pp.y;
	ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "sendXYtoTF gTest --> px: "<< px <<" py: "<< py <<" test_angle: "<< test_angle*180/M_PI<< "*");
}



tf::StampedTransform listenTF(string coord_center, string node_name, ros::NodeHandle & nodeh)
{
	static   tf::TransformListener listener;
	tf::StampedTransform transform;
	
	do
	{
		sendXYToTF(nodeh, "world", "gTest");
    	try{
	  		ros::Time now = ros::Time::now();	
	 		listener.waitForTransform(coord_center, node_name, now, ros::Duration(2.0) );
      		listener.lookupTransform(coord_center, node_name, ros::Time(0), transform);
			success_listen = true;
    	}
    	catch (tf::TransformException &ex) {
	 		ROS_INFO_STREAM("--Catch Error--");
      		ROS_ERROR("%s",ex.what());
      		ros::Duration(1.0).sleep();
      		success_listen =false;
    	}
		sendXYToTF(nodeh, "world", "gTest");
	}while(!success_listen);
	
	
	return transform;
}


void move(ros::Publisher &pb, float sp, ros::NodeHandle & nodeh ) // работает 
{
	gazebo_msgs::GetModelState ms = getSate( nodeh , (std::string) "world" , (std::string) "gTest"  );
	
	gazebo_msgs::ModelState msg;
    msg.model_name = "gTest";
	msg.pose = ms.response.pose;
	msg.twist = ms.response.twist;
    msg.twist.linear.x=sp;
    pb.publish(msg);
	ros::spinOnce();
    sleep(2.0);
	
}


void turn(ros::Publisher &pb, float angle, ros::NodeHandle & nodeh ) //ок
{
	gazebo_msgs::GetModelState ms = getSate( nodeh , (std::string) "world" , (std::string) "gTest"  );
	
	
	gazebo_msgs::ModelState msg;
    msg.model_name = "gTest";
	msg.pose = ms.response.pose;
	msg.twist = ms.response.twist;
    msg.twist.angular.z =angle;
    pb.publish(msg);
    ros::spinOnce();
	sleep(2.0);
	
	
}


int main(int argc, char **argv) 
{
	
	ros::init(argc,argv,"gTest");
	ros::NodeHandle nh;
	
	if (argc != 2){ROS_ERROR("need name as argument"); return -1;}
    name = argv[1];
	ROS_INFO_STREAM("name: "<< name);
	
	srand(time(NULL));
	px= rand()%20-10;
	py= rand()%20-10;
	
	//service running ***********************************************************************
	ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot =  nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;
 
    ifstream fin("/home/marc/.gazebo/models/pioneer3at/model.sdf");
 
    string model;
    string buf;
    while(!fin.eof()){
        getline(fin, buf);
        model += buf + "\n";
    }
    srv.request.model_xml = model;
    srv.request.model_name = "gTest";
    geometry_msgs::Pose pose;
	pose.position.x= px;
    pose.position.y= py;
	pose.position.z=0;
	pose.orientation.x= 0;
	pose.orientation.y= 0;
	pose.orientation.z= 0;
	pose.orientation.w= 1;
	//tf::Quaternion q;
	//q.setRPY(0, 0, M_PI/2);
    srv.request.initial_pose = pose;
    add_robot.call(srv);
	//running end *****************************************************************************
	
	
	
	ros::Publisher pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
	ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("/gTest/cmd_vel", 10);

	
	
	int i=0;
	ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "found -- px: "<< px <<" py: "<< py);
	
	ros::Rate rate(5);
	while(ros::ok())
	{
		sendXYToTF(nh, "world", name);
		move(pub,0.2,nh);
		//move1(pub_vel,5,rate);
		turn(pub, M_PI*2, nh);
		
		i++;
		ros::spinOnce();
		
		//rate.sleep();
	}
	
	
	return 0;
}