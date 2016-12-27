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
bool come;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	come = true;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "C3Po");

	ros::NodeHandle n;
	string model;
	string buf;
	float d = 0.1;
	fl = false;
	come = false;
	ros::Publisher pubR2D2 = n.advertise<std_msgs::String>("R2D2/cmd", 10);
	ros::Subscriber sub = n.subscribe("C3Po/cmd", 10, chatterCallback);

	ros::Rate loop_rate(100);

	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::TransformListener listener;
	tf::StampedTransform tr;

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
	srv.request.model_name = "C3Po";
	geometry_msgs::Pose pose;
	srv.request.initial_pose = pose;
	add_robot.call(srv);

	ros::Publisher pub =
	    n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
	sleep(1.0);

	gazebo_msgs::ModelState msg;
	msg.model_name = "C3Po";
	msg.pose.position.x = 15;
	msg.pose.position.y = 5;
	pub.publish(msg);

	float start_x = msg.pose.position.x;
	float start_y = msg.pose.position.y;


	while (ros::ok())
	{

		if (pubR2D2.getNumSubscribers() == 0)
		{
			sleep(1.0);
			continue;
		}

		transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "C3Po/pose"));

		if (!fl)
		{

			try
			{
				listener.waitForTransform("/C3Po/pose", "/R2D2/pose", ros::Time::now(), ros::Duration(0.5));
				listener.lookupTransform("/C3Po/pose", "/R2D2/pose", ros::Time(0), tr);
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s", ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}

			float tx = tr.getOrigin().x();
			float ty = tr.getOrigin().y();

			if (fabs(tx) < 0.05 && fabs(ty) < 0.05)
				//if (fabs(msg.pose.position.x - tr.getOrigin().x() < 0.05) && fabs(msg.pose.position.y - tr.getOrigin().y()) < 0.05)
			{
				std_msgs::String msg;
				msg.data = "!";
				fl = true;

				pubR2D2.publish(msg);
				continue;
			}

			float factor = 1000;

			/*float dx = fabs(msg.pose.position.x - tr.getOrigin().x()) / factor;
			dx *= msg.pose.position.x < tr.getOrigin().x() ? 1 : -1;
			float dy = fabs(msg.pose.position.y - tr.getOrigin().y()) / factor;
			dy *= msg.pose.position.y < tr.getOrigin().y() ? 1 : -1;
			*/
			float dx = fabs(tx) / factor;
			dx *= tx > 0 ? 1 : -1;
			float dy = fabs(ty) / factor;
			dy *= ty > 0 ? 1 : -1;

			for (int i = 0; i < factor; i++ ) {
				msg.pose.position.x += dx;
				msg.pose.position.y += dy;

				pub.publish(msg);

				loop_rate.sleep();

			}
		}
		else
		{
			float factor = 1000;

			float dx = fabs(msg.pose.position.x - start_x) / factor;
			dx *= msg.pose.position.x < start_x ? 1 : -1;
			float dy = fabs(msg.pose.position.y - start_y) / factor;
			dy *= msg.pose.position.y < start_y ? 1 : -1;

			for (float step = 0; step < factor; step++)
			{
				msg.pose.position.x += dx;
				msg.pose.position.y += dy;

				pub.publish(msg);

				transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
				tf::Quaternion q;
				q.setRPY(0, 0, 0);
				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "C3Po/pose"));

				loop_rate.sleep();

			}

			std_msgs::String msg;
			msg.data = "#";
			pubR2D2.publish(msg);
			sleep(2);
			return 0;
		}

		ros::spinOnce();
	}

	return 0;
}


