#include <ros/ros.h>
#include <visualization_msgs/Marker.h>  
#include "geometry_msgs/Point.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <string>
#include <cstring>

#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include <fstream>
 #include <geometry_msgs/Pose2D.h>


#define _USE_MATH_DEFINES // for C++
#include <cmath>


using namespace std;

float start_x=7, start_y=-7;
string name ;
float px, py;
bool hasFound=false;

float rescuer_angle, alpha;
float dx, dy;
const float incr = 0.0174532925; // 1* -> radian;
ros::ServiceClient srv_client ;
bool success_listen;
float mspeed=0.3;
float rspeed= 2*M_PI;

//std::string turtle_name==node_name;



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
  rescuer_angle=yaw;
	ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << " rescuer_angle: "<< rescuer_angle);
  if (rescuer_angle<0){
	  rescuer_angle+= (2*M_PI);
	  if (fabs(rescuer_angle-2*M_PI)<0.1 ) rescuer_angle =0.0;
  }
	else{
		if (fabs(rescuer_angle-2*M_PI)<0.1 ) rescuer_angle =0.0;
		//else if(rescuer_angle > 2*M_PI) rescuer_angle= fmod (rescuer_angle , ( 2*M_PI ));
	}
	
  //perfom x, y
  px=pp.x;
  py=pp.y;
	ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "sendXYtoTF gRescuer --> px: "<< px <<" py: "<< py <<" rescuer_angle: "<< rescuer_angle*180/M_PI<< "*");
}



tf::StampedTransform listenTF(string coord_center, string node_name, ros::NodeHandle & nodeh)
{
	static   tf::TransformListener listener;
	tf::StampedTransform transform;
	
	do
	{
		sendXYToTF(nodeh, "world", "gRescuer");
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
		sendXYToTF(nodeh, "world", "gRescuer");
	}while(!success_listen);
	
	
	return transform;
}


void move(ros::Publisher &pb, float sp, ros::NodeHandle & nodeh ) // работает 
{
	gazebo_msgs::GetModelState ms = getSate( nodeh , (std::string) "world" , (std::string) "gRescuer"  );
	
	gazebo_msgs::ModelState msg;
    msg.model_name = "gRescuer";
	msg.pose = ms.response.pose;
	msg.twist = ms.response.twist;
    msg.twist.linear.x=sp;
    pb.publish(msg);
	ros::spinOnce();
    sleep(2.0);
	
}


void turn(ros::Publisher &pb, float angle, ros::NodeHandle & nodeh ) //ок
{
	gazebo_msgs::GetModelState ms = getSate( nodeh , (std::string) "world" , (std::string) "gRescuer"  );
	
	
	gazebo_msgs::ModelState msg;
    msg.model_name = "gRescuer";
	msg.pose = ms.response.pose;
	msg.twist = ms.response.twist;
    msg.twist.angular.z =angle;
    pb.publish(msg);
    ros::spinOnce();
	sleep(2.0);
	
	
}


void turnToLost(ros::Publisher &pb, ros::NodeHandle & nodeh )
{
	
	ros::spinOnce();
	tf::StampedTransform transf = listenTF( "gRescuer", "gLost", nodeh);
	dx= transf.getOrigin().x();
	dy= transf.getOrigin().y();
	ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "turnToLost -> dx: "<< dx <<" dy: "<< dy);
	
	if(dx>0 && dy>=0) alpha= atan(fabs(dy/dx));
	if(dx==0 && dy>0) alpha= M_PI/2;
	if(dx<0 && dy>=0) alpha= M_PI - atan(fabs(dy/dx));
	if(dx<0 && dy<=0) alpha= M_PI + atan(fabs(dy/dx));
	if(dx>0 && dy<=0) alpha= 2*M_PI - atan(fabs(dy/dx));
	if(dx==0 && dy<0) alpha= 3*M_PI/2;

	
	float dth=rescuer_angle-alpha;
	while(fabs(rescuer_angle-alpha)>(3*incr))
	{
		turn(pb, 2*M_PI, nodeh);
		ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "turnToLost -> alpha: "<< alpha <<" rescuer_angle: "<< rescuer_angle);
		sendXYToTF( nodeh, "world" , "gRescuer");
	}	
}

void turnToStart(ros::Publisher &pb, ros::NodeHandle & nodeh )
{
	
	ros::spinOnce();
	tf::StampedTransform transf = listenTF( "gRescuer", "world", nodeh);
	dx= transf.getOrigin().x();
	dy= transf.getOrigin().y();
	ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "turnToStart -> dx: "<< dx <<" dy: "<< dy);
	
	if(dx>0 && dy>=0) alpha= atan(fabs(dy/dx));
	if(dx==0 && dy>0) alpha= M_PI/2;
	if(dx<0 && dy>=0) alpha= M_PI - atan(fabs(dy/dx));
	if(dx<0 && dy<=0) alpha= M_PI + atan(fabs(dy/dx));
	if(dx>0 && dy<=0) alpha= 2*M_PI - atan(fabs(dy/dx));
	if(dx==0 && dy<0) alpha= 3*M_PI/2;

	
	float dth=rescuer_angle-alpha;
	while(fabs(rescuer_angle-alpha)>(3*incr))
	{
		turn(pb, 2*M_PI, nodeh);
		ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "turnToStart -> alpha: "<< alpha <<" rescuer_angle: "<< rescuer_angle);
		sendXYToTF( nodeh, "world" , "gRescuer");
	}
}

int getMoveDirection()
{
	if(alpha>0 && alpha<M_PI)
	{
		 return -1;
	}
	else
	{
		if(alpha>M_PI)
		{
			return 1;
		}
		else
		{
			if(alpha==0)
			{
				return -1;
			}
			else
			{
				if(alpha==M_PI) return 1;
			}
		}
	}
}


int main(int argc, char ** argv)
{
	ros::init(argc, argv, "gRescuer");
	ros::NodeHandle nh;
   
	if (argc != 2){ROS_ERROR("need name as argument"); return -1;};
    name = argv[1];
	ROS_INFO_STREAM("name: "<< name);
	
	
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
    srv.request.model_name = "gRescuer";
    geometry_msgs::Pose pose;
    pose.position.x=start_x;
    pose.position.y= start_y;
	  pose.position.z=0;
    srv.request.initial_pose = pose;
    add_robot.call(srv);
	//running end *****************************************************************************
	
	
	ros::Publisher pub1 =  nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/gRescuer/cmd_vel", 10);
	ros::Publisher back= nh.advertise<std_msgs::String>("back",10);
	
	std_msgs::String Message;
    Message.data = std::string("back");
	


	turnToLost(pub1, nh );

  
	ros::Rate rate(30);
	while(ros::ok())
	{
		sendXYToTF(nh, "world", "gRescuer");
		ros::spinOnce();
		
       	tf::StampedTransform transf = listenTF( "gRescuer", "gLost", nh);
	    float rx= transf.getOrigin().x();
	    float ry= transf.getOrigin().y(); 
		
		
	    if(!hasFound && (fabs(rx)>1.4 || fabsf(ry)>1.4) )
		{
			
       		//move(pub1, getMoveDirection()* mspeed,  nh);
			move(pub1, -0.3,  nh);
			continue;
		}
		
		
		if(hasFound && (fabs(px)>1.4 && fabs(py)>1.4 ) )
		{
			move(pub1, getMoveDirection()* mspeed,  nh);
			continue;
		}
		
		

		
		if( !hasFound && fabs(rx)<=1.4 && fabsf(ry)<=1.4)
		{
			back.publish(Message);
			ros::spinOnce();
			hasFound= true;
			sleep(3);
			turnToStart(pub1, nh );
			continue;
		}
		
	   ros::spinOnce();	
	   rate.sleep();
    }


	return 0;
}