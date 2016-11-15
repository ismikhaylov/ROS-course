#include <ros/ros.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <cmath>  

using namespace std;

class Searcher {
private:
	//base info.
	string name;
	ros::NodeHandle &nh;
	int id;
	double x, y, z;

	string nameWalker;

	//for visualization.
	ros::Publisher pub;
	gazebo_msgs::ModelState robotState;


	void move(const std::string& targetFrame, const std::string& sourceFrame) {
		std::cout << "The robot searcher (" << name << ") begin to search (" << sourceFrame << ")" << std::endl;

		tf::TransformListener listener;
		float thresholdX = 0.75f, thresholdY = 0.75f;

		ros::Rate rate(1);
		while (nh.ok()) {
			tf::StampedTransform transform;
			try {
				listener.lookupTransform(targetFrame, sourceFrame, ros::Time(0), transform);
			}
			catch (tf::TransformException &e) {
				ROS_ERROR("%s", e.what());
				broadcastPosition();
				ros::Duration(1.0).sleep();
				continue;
			}

			x += sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().x(), 2)) / 2.0 * (transform.getOrigin().x() < 0) ? -1.0 : 1.0;
			y += sqrt(pow(transform.getOrigin().y(), 2) + pow(transform.getOrigin().y(), 2)) / 2.0 * (transform.getOrigin().y() < 0) ? -1.0 : 1.0;


			cout << "Searcher fabs(" << fabs(transform.getOrigin().x()) << ", " << fabs(transform.getOrigin().y()) << ")" << endl;		
			if (fabs(transform.getOrigin().x()) < thresholdX 
				&& fabs(transform.getOrigin().y()) < thresholdY)
				break;

			broadcastPosition();
			repaint();
			
			rate.sleep();
		}
	}

	void cry(const std::string& msg) {
		std_msgs::String s;
		s.data = msg;
	  	ros::Publisher pubMsg = nh.advertise<std_msgs::String>("/voice", 100);
	 
	 	cout << "Cry message: (" << msg << ")" << endl;
	  	ros::Rate loop_rate(1);
	  	for (int i = 0; i < 3; i++) {
	        pubMsg.publish(s);
	        loop_rate.sleep();
	    }
	}

	void broadcastPosition() {
		//for broadcast.
		static tf::TransformBroadcaster br;
		tf::Transform transform;

		cout << "Searcher (" << name <<") went to x:" << x << " y:" << y << endl;

		transform.setOrigin(tf::Vector3(x, y, 0.0));
		tf::Quaternion q;
		q.setRPY(0, 0, z);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
	}

	void repaint() {
		// marker.pose.position.x = x;
		// marker.pose.position.y = y;
		// marker.pose.position.z = z;
		// marker_pub.publish(marker);

		robotState.pose.position.x = x;
	    robotState.pose.position.y = y;
	    robotState.pose.position.z = z;
	    pub.publish(robotState);
	}


public:
	Searcher(ros::NodeHandle &nh, string name, string nameWalker) : nh(nh), name(name), nameWalker(nameWalker) {
		id = 2;
		x = y = z = 0;

		pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
	}

	void search() {
		move(name, nameWalker);
		cry(name);	
	}

	void comeback() {
		move(name, "world");
		cry("stop");
	}

	void initVisualisation() {
		cout << "init visualization" << endl;

		ros::service::waitForService("gazebo/spawn_sdf_model");
	    ros::ServiceClient add_robot = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	    gazebo_msgs::SpawnModel srv;
	 
	    ifstream fin("/home/user/.gazebo/models/robonaut/model.sdf");
 

		string model;
	    string buf;
	    while(!fin.eof()){
	        getline(fin, buf);
	        model += buf + "\n";
	    }
	    srv.request.model_xml = model;
	    srv.request.model_name = name;
	    geometry_msgs::Pose pose;
	    srv.request.initial_pose = pose;
	    add_robot.call(srv);

	    //state...
	    robotState.model_name = name;
	    robotState.pose.position.x = 0.0;
		robotState.pose.position.y = 0.0;
		robotState.pose.position.z = 0.0;

		// marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
		// marker.header.frame_id = name;
		// marker.header.stamp = ros::Time::now();
		// marker.ns = name + "_namespace";
		// marker.action = visualization_msgs::Marker::ADD;
		// marker.id = id;
		// marker.type = visualization_msgs::Marker::SPHERE;

		// marker.pose.orientation.x = 0.0;
		// marker.pose.orientation.y = 0.0;
		// marker.pose.orientation.z = 0.0;
		// marker.pose.orientation.w = 0.0;

		// marker.scale.x = 1.0;
		// marker.scale.y = 1.0;
		// marker.scale.z = 1.0;
		
		// marker.pose.position.x = 0.0;
		// marker.pose.position.y = 0.0;
		// marker.pose.position.z = 0.0;

		// marker.color.g = 1.0f;
		// marker.color.a = 1.0;

		broadcastPosition();
		repaint();
		sleep(3);
	}

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_searcher");


	if (argc < 2) {
		ROS_ERROR("Not specified robot's name as an argument!");
		return -1;
	}

	if (argc < 3) {
		ROS_ERROR("Not specified walker robot's name as an argument!");
		return -1;
	}

	ros::NodeHandle nh;
	Searcher searcher(nh, argv[1], argv[2]);
	searcher.initVisualisation();
	searcher.search();
	searcher.comeback();

	return 0;
}