#ifndef __BATROBOT__
#define __BATROBOT__


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <cmath>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#define steplength 0.05
#define min_dist 2.0

class Robot {
public:
	Robot(std::string name, int id, float r, float g, float b, float x, float y);
	void move(float x, float y);
	bool isCloseEnough (std::string name);
	float m_anotherX;
	float m_anotherY;
	
private:
	void sendTransform(float x, float y);
	std::string m_name;
	visualization_msgs::Marker m_marker;
	ros::NodeHandle m_nh;
	ros::Publisher m_pub;
	gazebo_msgs::ModelState m_msg;
};


#endif