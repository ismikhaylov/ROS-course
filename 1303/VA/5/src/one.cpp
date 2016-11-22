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

int main(int argc, char ** argv)//20
{
	ros::init(argc, argv, "one");

        ros::NodeHandle n;

	
	fl = false;

	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::TransformListener listener;
	tf::StampedTransform tr;

	ros::Publisher pub = n.advertise<std_msgs::String>("two/cmd", 10);
	ros::Rate loop_rate(50);

        ros::service::waitForService("gazebo/spawn_sdf_model");
  //40 
     ros::ServiceClient add_robot = 
             n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
        gazebo_msgs::SpawnModel srv;
 
        ifstream fin("/home/ckobar/.gazebo/models/pioneer2dx/model.sdf");
 
        string model;
        string buf;
        while(!fin.eof()){
            getline(fin, buf);
            model += buf + "\n";
        }
        srv.request.model_xml = model;
        srv.request.model_name = "one";
        geometry_msgs::Pose pose;
        srv.request.initial_pose = pose;
        add_robot.call(srv);
        //Spawning finished
 
//60
        ros::Publisher pubgaz = 
            n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
        sleep(1.0);
 
        gazebo_msgs::ModelState msg;
        msg.model_name = "one";
        msg.pose.position.x = 6;
	msg.pose.position.y = 4;
        pubgaz.publish(msg);

	float start_x = msg.pose.position.x;
	float start_y = msg.pose.position.y;

	while (ros::ok())
	{

		if (pub.getNumSubscribers() == 0)
		{			
			continue;
		}

		transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "one/pose"));

		if (!fl)
		{

			listener.waitForTransform("/one/pose", "/two/pose", ros::Time::now(), ros::Duration(0.5));
			listener.lookupTransform("/one/pose", "/two/pose", ros::Time(0), tr);
			
			if (fabs(tr.getOrigin().x()) < 0.1 && fabs(tr.getOrigin().y()) < 0.1)
			{
				std_msgs::String msg;
				msg.data = "ye";
				fl = true;

				pub.publish(msg);
				continue;
			}

			float dx = fabs(tr.getOrigin().x()) / 200;
			dx *= tr.getOrigin().x() > 0 ? 1 : -1;
			float dy = fabs(tr.getOrigin().y()) / 200;
			dy *= tr.getOrigin().y() > 0 ? 1 : -1;

			for (int i = 0; i < 200; i++ ) {
				msg.pose.position.x += dx;
				msg.pose.position.y += dy;

				pubgaz.publish(msg);

				loop_rate.sleep();

			}
		}
		else
		{
			float dx = fabs(msg.pose.position.x - start_x) / 200;
			dx *= msg.pose.position.x < start_x ? 1 : -1;
			float dy = fabs(msg.pose.position.y - start_y) / 200;
			dy *= msg.pose.position.y < start_y ? 1 : -1;
			cout << "msg.pose.position.x: " << msg.pose.position.x << "\nstart_x: " << start_x << "\nmsg.pose.position.y: " << msg.pose.position.y << "\nstart_y: " << start_y << endl;
			while (msg.pose.position.x != start_x && msg.pose.position.y != start_y)
			{
				cout << "msg.pose.position.x: " << msg.pose.position.x << "\nstart_x: " << start_x << "\nmsg.pose.position.y: " << msg.pose.position.y << "\nstart_y: " << start_y << endl;
				msg.pose.position.x += dx;
				msg.pose.position.y += dy;

				pubgaz.publish(msg);

				transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
				tf::Quaternion q;
				q.setRPY(0, 0, 0);
				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "one/pose"));

				loop_rate.sleep();

			}

			return 0;
		}

		ros::spinOnce();
	}

	return 0;
}

