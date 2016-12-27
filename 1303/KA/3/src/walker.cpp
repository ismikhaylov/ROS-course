#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <ros/console.h>
#include <string> 
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
	ros::Publisher marker_pub;
	visualization_msgs::Marker marker;


	void follow() {
		std::cout << "The robot walker begin to follow" << std::endl;

		tf::TransformListener listener;
		float thresholdX = 0.5f, thresholdY = 0.5f;

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
			x += (double) (rand() % 10 - 5) / 2.0;
			y += (double) (rand() % 10 - 5) / 2.0;
			z = 0.0;

			broadcastPosition();
			repaint(); //repaint marker.

			ros::spinOnce();
			loop_rate.sleep();
		}

		follow();
	}

	void initVisualisation() {
		marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
		marker.header.frame_id = name;
		marker.header.stamp = ros::Time::now();
		marker.ns = name + "_namespace";
		marker.action = visualization_msgs::Marker::ADD;
		marker.id = id;
		marker.type = visualization_msgs::Marker::SPHERE;

		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 0.0;

		marker.scale.x = 1.0;
		marker.scale.y = 1.0;
		marker.scale.z = 1.0;
		
		marker.pose.position.x = 0.0;
		marker.pose.position.y = 0.0;
		marker.pose.position.z = 0.0;

		marker.color.r = 1.0f;
		marker.color.a = 1.0;
	}

	void repaint() {
		marker.pose.position.x = x;
		marker.pose.position.y = y;
		marker.pose.position.z = z;
		marker_pub.publish(marker);
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