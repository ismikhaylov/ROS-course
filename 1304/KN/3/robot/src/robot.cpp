#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <turtlesim/Pose.h>
#include <stdlib.h>
#include <cmath> 


bool fl;
std::string robotName;

float curPositionX;
float curPositionY;

const float DELTA = 0.1;
float ID;

static tf::TransformBroadcaster* br;
static ros::Publisher marker_pub;

void drawPosition(float x, float y, const ros::Publisher& marker_pub);
void animateMove(float x, float y);
void chatterCallback(const geometry_msgs::Point& msg);
void sendTransf(float x, float y);

int main(int argc, char ** argv) {
	if (argc != 4) {
		ROS_ERROR("need robot name and start coordinates as arguments"); return -1;
	};
	robotName = argv[1];
	curPositionX = atoi(argv[2]);
	curPositionY = atoi(argv[3]);
	srand (time(NULL));
	char* nameForId = argv[1] + 1;
	ID = atoi(nameForId);

	ros::init(argc, argv, robotName);

	br = new tf::TransformBroadcaster();

	ros::NodeHandle n;
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	ros::Subscriber sub = n.subscribe(robotName + "/pose", 10, chatterCallback);

	ros::Rate loop_rate(100);

	while (ros::ok()) {
		ros::spinOnce();
		sendTransf(curPositionX, curPositionY);
		drawPosition(curPositionX, curPositionY, marker_pub);
		loop_rate.sleep();
	}

	return 0;
}


void animateMove(float x, float y) {
	ros::Rate loop_rate(10);
	while ((std::abs(x) > DELTA) || (std::abs(y) > DELTA)) {
		std::ostringstream logmsg;
		logmsg << robotName << "[" << ID << "]: ANIM x = " << x << " y = " << y << " (";

		if (std::abs(x) > DELTA) {
			curPositionX += x * DELTA;
			x -= x * DELTA;
			logmsg << " x -= " << x * DELTA;
		}
		if (std::abs(y) > DELTA) {
			curPositionY += y * DELTA;
			y -= y * DELTA;
			logmsg << " y -= " << y * DELTA;
		}

		logmsg << " NOW x = " << x << " y = " << y;

		ROS_INFO(logmsg.str().c_str());
		drawPosition(curPositionX, curPositionY, marker_pub);
		sendTransf(curPositionX, curPositionY);
		ros::spinOnce();
		loop_rate.sleep();
	}
	ROS_INFO("END ANIM");
}

void sendTransf(float x, float y) {
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(x, y, 0.0) );
	br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robotName));

	std::ostringstream logmsg;
	logmsg << robotName << "[" << ID << "]: SEND TF (" << x << ", " << y << ")";
	ROS_INFO(logmsg.str().c_str());
}

void drawPosition(float x, float y, const ros::Publisher& marker_pub) {
	std::ostringstream logmsg;
	logmsg << robotName << "[" << ID << "]: cur position (" << x << ", " << y << ")";
	ROS_INFO(logmsg.str().c_str());

	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();

	marker.ns = "basic_shapes";
	marker.id = ID;

	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = 0;

	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;

	if (ID == 1) {
		marker.color.r = 1.0f;
	} else {
		marker.color.r = 0.0f;
	}

	if (ID == 2) {
		marker.color.g = 1.0f;
	} else {
		marker.color.g = 0.0f;
	}
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);
}

void chatterCallback(const geometry_msgs::Point& msg) {
	std::ostringstream logmsg;
	logmsg << robotName << ": NEW POSITION (" << msg.x << ", " << msg.y << ")";
	ROS_INFO(logmsg.str().c_str());

	animateMove(msg.x, msg.y);
}
