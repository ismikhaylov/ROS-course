#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <turtlesim/Pose.h>
#include <cmath> 

const float STEP = 1;
const float DELTA = 0.1;
bool found = false;

void moveFinder(tf::TransformListener& listener, ros::Publisher& finder);
void moveLost(tf::TransformListener& listener, ros::Publisher& lost);

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "Dispatcher");
	ros::NodeHandle node;
	ros::Publisher finder = node.advertise<geometry_msgs::Point>("a1/pose", 10);
	ros::Publisher lost = node.advertise<geometry_msgs::Point>("a2/pose", 10);

	tf::TransformListener listener;
	ros::Rate rate(2.0);
	while (node.ok()) {
		
		moveFinder(listener, finder);
		moveLost(listener, lost);

	    rate.sleep();
	}
}

void moveFinder(tf::TransformListener& listener, ros::Publisher& finder) {
	std::string dist = found ? "/a3" : "/a2";

	tf::StampedTransform transform;
    try {
      listener.lookupTransform(dist, "/a1", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }

    std::ostringstream logmsg;

    if ((std::abs(transform.getOrigin().x()) < DELTA) && (std::abs(transform.getOrigin().y()) < DELTA)) {
    	found = true;
    	logmsg << " FOUND! ";
    }

    logmsg << "GET " << transform.getOrigin().x() << " " << transform.getOrigin().y();

    geometry_msgs::Point vel_msg;
    if (transform.getOrigin().x() < 0) {
    	vel_msg.x = (transform.getOrigin().x() < STEP)? STEP : transform.getOrigin().x();
    } else {
    	vel_msg.x = (transform.getOrigin().x() < STEP)? transform.getOrigin().x() : (-1) * STEP;
    }

    if (transform.getOrigin().y() < 0) {
    	vel_msg.y = (transform.getOrigin().y() < STEP)? STEP : transform.getOrigin().y();
    } else {
    	vel_msg.y = (transform.getOrigin().y() < STEP)? transform.getOrigin().y() : (-1) * STEP;
    }

    finder.publish(vel_msg);
	logmsg << "\nSEND " << vel_msg;
	ROS_INFO(logmsg.str().c_str());
}

void moveLost(tf::TransformListener& listener, ros::Publisher& lost) {
	if (!found) {
		return;
	}

	tf::StampedTransform transform;
    try {
      listener.lookupTransform("/a1", "/a2", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }

    std::ostringstream logmsg;

    logmsg << "[LOST] GET " << transform.getOrigin().x() << " " << transform.getOrigin().y();

    geometry_msgs::Point vel_msg;
    if (transform.getOrigin().x() < 0) {
    	vel_msg.x = (transform.getOrigin().x() < STEP)? STEP : transform.getOrigin().x();
    } else {
    	vel_msg.x = (transform.getOrigin().x() < STEP)? transform.getOrigin().x() : (-1) * STEP;
    }

    if (transform.getOrigin().y() < 0) {
    	vel_msg.y = (transform.getOrigin().y() < STEP)? STEP : transform.getOrigin().y();
    } else {
    	vel_msg.y = (transform.getOrigin().y() < STEP)? transform.getOrigin().y() : (-1) * STEP;
    }

    lost.publish(vel_msg);
	logmsg << "\nSEND " << vel_msg;
	ROS_INFO(logmsg.str().c_str());
}
