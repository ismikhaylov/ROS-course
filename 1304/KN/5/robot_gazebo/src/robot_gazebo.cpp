#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <turtlesim/Pose.h>
#include <stdlib.h>
#include <cmath> 
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>

using namespace std;

bool fl;
std::string robotName;

float curPositionX;
float curPositionY;

const float DELTA = 0.05;
float ID;

static tf::TransformBroadcaster* br;
static ros::Publisher view_pub;

void drawPosition(float x, float y);
void animateMove(float x, float y);
void chatterCallback(const geometry_msgs::Point& msg);
void sendTransf(float x, float y);
void spawnRobot();

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
	ros::Subscriber sub = n.subscribe(robotName + "/pose", 10, chatterCallback);
	view_pub = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
	// marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	spawnRobot();

	ros::Rate loop_rate(100);

	while (ros::ok()) {
		ros::spinOnce();
		sendTransf(curPositionX, curPositionY);
		drawPosition(curPositionX, curPositionY);
		loop_rate.sleep();
	}

	return 0;
}

void spawnRobot() {
    ros::NodeHandle node;
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot = 
             node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;
 
    ifstream fin("/home/kodoo/.gazebo/models/pioneer2dx/model.sdf");
 
    string model;
    string buf;
    while(!fin.eof()){
        getline(fin, buf);
        model += buf + "\n";
    }
    srv.request.model_xml = model;
    srv.request.model_name = robotName;
    geometry_msgs::Pose pose;
    srv.request.initial_pose = pose;
    add_robot.call(srv);

    sleep(1.0);
 
    gazebo_msgs::ModelState msg;
    msg.model_name = robotName;

    msg.pose.position.x = curPositionX;
    msg.pose.position.y = curPositionY;

    view_pub.publish(msg);
    sleep(0.3);
    ros::spinOnce();
    //Spawning finished
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
		drawPosition(curPositionX, curPositionY);
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

void drawPosition(float x, float y) {
	std::ostringstream logmsg;
	logmsg << robotName << "[" << ID << "]: cur position (" << x << ", " << y << ")";
	ROS_INFO(logmsg.str().c_str());

	gazebo_msgs::ModelState msg;
    msg.model_name = robotName;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    view_pub.publish(msg);
	
}

void chatterCallback(const geometry_msgs::Point& msg) {
	std::ostringstream logmsg;
	logmsg << robotName << ": NEW POSITION (" << msg.x << ", " << msg.y << ")";
	ROS_INFO(logmsg.str().c_str());

	animateMove(msg.x, msg.y);
}
