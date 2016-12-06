//https://www.clearpathrobotics.com/2014/09/ros-101-creating-node/

#include "ros/ros.h"
#include <stdlib.h>
#include <iostream>
#include <string>
#include <cstring>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>


#define _USE_MATH_DEFINES // for C++
#include <cmath>

bool go_right ;
bool go_left ;
bool go_top ;
bool go_back ;
bool stop=false;
float X_end, Y_end;
float robot_posX,robot_posY, robot_angle;
float alpha;
float dx, dy, dth;
float speed= 0.25;
const float incr = 0.0174532925; // 1* -> radian;

void readScan(const sensor_msgs::LaserScan &lscan)
{
	
	go_right= true;
	go_top = true;
	go_left=true;
	ROS_INFO_STREAM(std::setprecision(8)<<std::fixed << "angle_min: " <<lscan.angle_min <<" angle_max: "<< lscan.angle_max<<" incr:"<<lscan.angle_increment );
	for(int i=0 ; i < 4 ; i++)
	{
		if( lscan.ranges[i]>0.5*2)
		{
			go_right= go_right && true;
		}
		else
		{
			go_right= go_right && false;
		}
		
	}
	
	for(int i=lscan.ranges.size()/4; i<3*lscan.ranges.size()/4  ; i++)
	{
		if( lscan.ranges[i]>0.5)
		{
			go_top= go_top && true;
		}
		else
		{
			go_top= go_top && false;
		}
	}
		
	for(int i=lscan.ranges.size()-4;  i<lscan.ranges.size(); i++)
	{
		if(lscan.ranges[i]>0.5*2)
		{
			go_left=go_left && true;
		}
		else
		{
			go_left=go_left && false;
		}
	}	
}


void readOdometry(const nav_msgs::Odometry & od)
{
	double roll, pitch, yaw;
	robot_posX= od.pose.pose.position.x;
	robot_posY= od.pose.pose.position.y;
	tf::Quaternion q(od.pose.pose.orientation.x, od.pose.pose.orientation.y, od.pose.pose.orientation.z, od.pose.pose.orientation.w );
	tf::Matrix3x3 matr(q);
	matr.getRPY(roll, pitch, yaw);
	robot_angle=yaw;
	
	//****************
	dx=X_end - robot_posX;
	dy=Y_end -robot_posY ;
	if(fabs(dx)<=0.5 && fabs(dy)<= 0.5) stop= true;
	
	alpha= atan(dy/dx);
	dth=robot_angle-alpha;
	ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "robot position x: " <<robot_posX <<" y: "<< robot_posY<< " r_angle="<<robot_angle); //*****
	ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "dX: " <<dx <<" dY: "<< dy <<" dth: "<<dth);
	
}


void turn(ros::Publisher &pb, float angle, ros::Rate &rate )
{
	geometry_msgs::Twist msg;
	msg.angular.z =angle;
	pb.publish(msg);
	rate.sleep();
	ros::spinOnce();
	
}

void turnToX_positive(ros::Publisher &pb)
{
	ros::spinOnce();
	float dd=robot_angle;
	ros::Rate r(2);
	while(fabs(robot_angle)>(1.5*incr))
	{
		turn(pb, -dd/3, r);
	}
}

void turnToX_negative(ros::Publisher &pb)
{
	ros::spinOnce();
	float dd=robot_angle-M_PI;
	ros::Rate r(2);
	while(fabs(robot_angle-M_PI)>(1.5*incr))
	{
		turn(pb, -dd/3, r);
	}
}

void turnToY_positive(ros::Publisher &pb)
{
	ros::spinOnce();
	float dd=robot_angle-M_PI/2;
	ros::Rate r(2);
	while(fabs(robot_angle-M_PI/2)>(1.5*incr))
	{
		turn(pb, -dd/3, r);
	}
}

void turnToY_negative(ros::Publisher &pb)
{
	ros::spinOnce();
	float dd=robot_angle-3/2*M_PI;
	ros::Rate r(2);
	while(fabs(robot_angle-3/2*M_PI)>(1.5*incr))
	{
		turn(pb, -dd/3, r);
	}
}

void turnToEnd(ros::Publisher &pb)
{
	ros::spinOnce();
	float dd= dth;
	ros::Rate r(2);
	while(fabs(robot_angle-alpha)>(1.5*incr))
	{
		turn(pb, -dd/3, r);
	}	
}

void move(ros::Publisher &pb, float sp, ros::Rate &rate)
{
	geometry_msgs::Twist msg;
	msg.linear.x =sp;
	pb.publish(msg);
	rate.sleep();
	ros::spinOnce();
}

int main (int argc, char **argv )
{
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;
  
  ros::Publisher scan_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Subscriber sb_od = nh.subscribe("/base_pose_ground_truth", 1000, readOdometry);
  ros::Subscriber sb_sc = nh.subscribe("/base_scan", 1000, readScan);
  ros::Rate loop_rate(30);	

  X_end=1;
  Y_end= 2;
  
  ROS_INFO_STREAM(std::setprecision(8)<<std::fixed << "X: " <<X_end <<" Y: "<< Y_end<<" Pi="<< M_PI  );
 
  
  turnToEnd(scan_pub);
  while(ros::ok())
  {
	 
	 while(go_top && !stop )
	 {
		move(scan_pub, speed, loop_rate); 
	 }
	  
	 if(!go_top && dx>0 && dy> 0 && !stop)
	 {
		 turnToX_positive(scan_pub);
		
		 do{
			 move(scan_pub, speed, loop_rate);
		 }while(!go_left&& go_top && !stop);
		 //Обход препятствия 
		 float px=robot_posX;
		 float py=robot_posY;
		 while(robot_posX<px+0.6 && go_top && !stop)
		 {
			 move(scan_pub, speed, loop_rate);
		 }
		 
		 turnToY_positive(Marysa – Avec Toi);
		 
		 while(go_top && !stop )
	 	 {
			move(scan_pub, speed, loop_rate); 
	 	 }
	 }
	 else{
		 
		 if(!go_top && dx>0 && dy<0 && !stop)
		 {  
			turnToY_negative(scan_pub);
			 
			do{
				 move(scan_pub, speed, loop_rate);
		 	}while(!go_left && go_top && !stop);
			 
			//обход припятствия 
		    float px=robot_posX;
		    float py=robot_posY;
		    while(robot_posY>py-0.6 && go_top && !stop)
		    {
			    move(scan_pub, speed, loop_rate);
		    }
			 
			turnToX_positive(scan_pub); 
			
		    while(go_top && !stop )
	 	    {
			  move(scan_pub, speed, loop_rate); 
	 	    }
			
			 
		 }
		 else{
			 
			 if(!go_top && dx<0 && dy>0 && !stop)
			 {
				turnToY_positive(scan_pub); 
				
				do{
					move(scan_pub, speed, loop_rate);
				}while(!go_left && go_top && !stop);
				//обход припятствия 
		        float px=robot_posX;
		        float py=robot_posY;
				while(robot_posY<py+0.6 && go_top && !stop)
		        {
			      move(scan_pub, speed, loop_rate);
		        }
				 
				turnToX_negative(scan_pub); 
				 
				while(go_top && !stop )
	 	    	{
			  		move(scan_pub, speed, loop_rate); 
	 	    	} 
				 
			 }
			 else{
				 if(!go_top && dx<0 && dy<0 && !stop)
				 {
					turnToX_negative(scan_pub);
					do{
					   move(scan_pub, speed, loop_rate);
				    }while(!go_left && go_top && !stop);
					//обход припятствия 
		            float px=robot_posX;
		            float py=robot_posY;
				    while(robot_posX>px-0.6 && go_top && !stop)
		            {
			      		 move(scan_pub, speed, loop_rate);
		            }
					 
					 
					turnToY_negative(scan_pub);
					 
					while(go_top && !stop )
	 	    	    {
			  		   move(scan_pub, speed, loop_rate); 
	 	    	    } 
				  
					 
				 }
			 }
			 
		 }
		 
		 
	 }
		 
	turnToEnd(scan_pub);
	loop_rate.sleep();  
  }
  
  
  return 0;
}