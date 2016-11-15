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

class Walker {
private:
	//base info.
	string name;
	ros::NodeHandle &nh;
	bool isFollow;
	int id;
	double x, y, z;

	string nameSearcher;

	//for visualization.
	ros::Publisher pub;
	gazebo_msgs::ModelState robotState;



	void follow() {
		std::cout << "The robot walker begin to follow" << std::endl;

		tf::TransformListener listener;

		ros::Rate rate(1);
		while (nh.ok() && isFollow) {
			tf::StampedTransform transform;
			try {
				listener.lookupTransform(name, nameSearcher, ros::Time(0), transform);
			}
			catch (tf::TransformException &e) {
				ROS_ERROR("%s", e.what());
				broadcastPosition();
				ros::Duration(1.0).sleep();
				continue;
			}


			x += transform.getOrigin().x();
			y += transform.getOrigin().y();

			broadcastPosition();
			repaint();
			
			ros::spinOnce();
			rate.sleep();
		}
	}


	void broadcastPosition() {
		//for broadcast.
		static tf::TransformBroadcaster br;
		tf::Transform transform;

		cout << "Walker (" << name <<") went to x:" << x << " y:" << y << endl;

		transform.setOrigin(tf::Vector3(x, y, 0.0));
		tf::Quaternion q;
		q.setRPY(0, 0, z);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
	}



public:
	Walker(ros::NodeHandle &nh, string name) : nh(nh), name(name) {
		isFollow = false;
		id = 1;
		x = y = z = 0;

		pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
		sleep(1.0);
	}


	string getName() {
		return name;
	}


	void handlerVoice(const std_msgs::String& str) {
		if (str.data != "stop") {
			nameSearcher = str.data;
			isFollow = true;
		} else isFollow = false;
		cout << "Hear a voice (" << str.data << ")" << endl;
	}


	ros::Subscriber listen() {
		return nh.subscribe("/voice", 100, &Walker::handlerVoice, this);
	}


	void walk() {
		ros::Rate loop_rate(1);
		while(ros::ok() && !isFollow) {
			x += (double) (rand() % 10 - 5) / 4.0;
			y += (double) (rand() % 10 - 5) / 4.0;
			z = 0.0;

			broadcastPosition();
			repaint(); //repaint state.

			ros::spinOnce();
			loop_rate.sleep();
		}

		follow();
	}


	void initVisualisation() {
		//spawning...
		ros::service::waitForService("gazebo/spawn_sdf_model");
	    ros::ServiceClient add_robot = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	    gazebo_msgs::SpawnModel srv;
	 
	    ifstream fin("/home/user/.gazebo/models/person_walking/model.sdf");
 

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
	}


	void repaint() {
	    robotState.pose.position.x = x;
	    robotState.pose.position.y = y;
	    robotState.pose.position.z = z;
	    pub.publish(robotState);
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "robot_walker");

	if (argc != 2) {
		ROS_ERROR("Not specified robot's name as an argument!");
		return -1;
	}

	ros::NodeHandle nh;
	Walker walker(nh, argv[1]);
	walker.initVisualisation();
	ros::Subscriber sub = walker.listen();
	walker.walk();

	return 0;
}