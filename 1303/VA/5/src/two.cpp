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
int chet = 1; 

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	fl = true;
}

int main(int argc, char ** argv)
{

	ros::init(argc, argv, "two");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::String>("one/cmd", 10);
	ros::Subscriber sub = n.subscribe("two/cmd", 10, chatterCallback);
	string model;
	string buf;

	ros::Rate loop_rate(50);

	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::TransformListener listener;
	tf::StampedTransform tr;

	ros::service::waitForService("gazebo/spawn_sdf_model");
	ros::ServiceClient add_robot =
	    n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	gazebo_msgs::SpawnModel srv;

	ifstream fin("/home/ckobar/.gazebo/models/pioneer2dx/model.sdf");


	while (!fin.eof() && fin) {
		getline(fin, buf);
		model += buf + "\n";
	}

	srv.request.model_xml = model;
	srv.request.model_name = "two";
	geometry_msgs::Pose pose;
	srv.request.initial_pose = pose;
	add_robot.call(srv);

	ros::Publisher pubgaz =
	    n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
	sleep(1.0);

	gazebo_msgs::ModelState msg;
	msg.model_name = "two";
	msg.pose.position.x = 5;
	msg.pose.position.y = 0;

	pubgaz.publish(msg);

	fl = false;
	int j = 1;
	bool ones = false;
	int check = 0;
	

	while (ros::ok())
	{
		j ++;
		
		transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "two/pose"));
		
		float old_tr_x = fabs(tr.getOrigin().x());
		float old_tr_y = fabs(tr.getOrigin().y());
		if (fl)
		{
			listener.waitForTransform("/two/pose", "/one/pose", ros::Time::now(), ros::Duration(0.5));
			listener.lookupTransform("/two/pose", "/one/pose", ros::Time(0), tr);

			float dx = fabs(tr.getOrigin().x()) / 30;
			dx *= tr.getOrigin().x() > 0 ? 1 : -1;
			float dy = fabs(tr.getOrigin().y()) / 30;
			dy *= tr.getOrigin().y() > 0 ? 1 : -1;

			cout << "fabs(tr.getOrigin().x()): " << fabs(tr.getOrigin().x()) << "\nfabs(tr.getOrigin().y()): " << fabs(tr.getOrigin().y()) << endl;
			cout << "\nold_tr_x: " << old_tr_x << "\nold_tr_y: " << old_tr_y << endl;			

			check+=1;
			msg.pose.position.x += dx;
			msg.pose.position.y += dy;

			pubgaz.publish(msg);
			loop_rate.sleep();
			if(check >=5 && old_tr_x == fabs( tr.getOrigin().x()) && old_tr_y == fabs(tr.getOrigin().y())){
				
				return 0;
				 for(int i = 0; i<50; i++)
				{

					msg.pose.position.x += dx;
					msg.pose.position.y += dy;

					pubgaz.publish(msg);
				}
				
				return 0;
			}


		}
		
		else
		{
			loop_rate.sleep();
			
			
		}

		ros::spinOnce();
		//loop_rate.sleep();
	}

	return 0;
}
