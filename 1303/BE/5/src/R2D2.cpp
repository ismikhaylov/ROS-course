#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"

using namespace std;
bool fl;
bool come = false;
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	if (msg->data == "#") {
		come = true;
		cout << "come == true" << endl;
	}
	if (msg->data == "!") {
		fl = true;
		cout << "fl == true" << endl;
	}
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "R2D2");

	ros::NodeHandle n;
	ros::Publisher pubR2D2 = n.advertise<std_msgs::String>("C3Po/cmd", 10);
	ros::Subscriber sub = n.subscribe("R2D2/cmd", 10, chatterCallback);

	ros::Rate loop_rate(100);

	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::TransformListener listener;
	tf::StampedTransform tr;


	float d = 0.1;
	fl = false;
	int i = 0;
	string model;
	string buf;

	ros::service::waitForService("gazebo/spawn_sdf_model");
	ros::ServiceClient add_robot =
	    n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	gazebo_msgs::SpawnModel srv;

	ifstream fin("/home/rina/.gazebo/models/pioneer2dx/model.sdf");


	while (!fin.eof() && fin) {
		getline(fin, buf);
		model += buf + "\n";
	}

	srv.request.model_xml = model;
	srv.request.model_name = "R2D2";
	geometry_msgs::Pose pose;
	srv.request.initial_pose = pose;
	add_robot.call(srv);

	ros::Publisher pub =
	    n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
	sleep(1.0);

	gazebo_msgs::ModelState msg;
	msg.model_name = "R2D2";
	msg.pose.position.x = 3;
	msg.pose.position.y = -3;

	pub.publish(msg);


	while (ros::ok())
	{

		transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "R2D2/pose"));


		if (fl)
		{

			try
			{
				listener.waitForTransform("/R2D2/pose", "/C3Po/pose", ros::Time::now(), ros::Duration(0.5));
				listener.lookupTransform("/R2D2/pose", "/C3Po/pose", ros::Time(0), tr);
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s", ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}

			float tx = tr.getOrigin().x();
			float ty = tr.getOrigin().y();


			float factor = 100;
			float dx = fabs(tx) / factor;
			dx *= tx > 0 ? 1 : -1;
			float dy = fabs(ty) / factor;
			dy *= ty > 0 ? 1 : -1;

			for (int i = 0; i < factor; i++ )
			{

				msg.pose.position.x += dx;
				msg.pose.position.y += dy;

				pub.publish(msg);
				loop_rate.sleep();
				cout << "at: " << msg.pose.position.x << "; " << msg.pose.position.y << " -- " << tx << "; " << ty << endl;
			}


			if (come)
			{
				return 0;
				for (int i = 0; i < factor; i++ )
				{

					msg.pose.position.x += dx;
					msg.pose.position.y += dy;

					pub.publish(msg);
					loop_rate.sleep();
				}
				return 0;
			}
		}
		else
		{
			loop_rate.sleep();
			cout << "wait" << endl;
		}

		ros::spinOnce();
	}

	return 0;
}