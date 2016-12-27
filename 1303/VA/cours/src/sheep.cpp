#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

using namespace std;

bool fl;
bool fl2 = false;
bool fl3 = false;
bool come = false;
std_msgs::String msgS;

struct pointS{
	float x;
	float y;
};

pointS stop1;
pointS stop2;
pointS stop3;
pointS stop4;
pointS stop5;

bool fls = false;
string wolf = "/wolf/pose";
int Arr[10];
void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	if(!fl3){
		//cout<<"arr"<<endl;
		int i = 0;
		// print all the remaining numbers
		for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
		{
			Arr[i] = *it;
			i++;
		
		}
		stop1.x = Arr[0];
		stop1.y = Arr[1];

		stop2.x = Arr[2];
		stop2.y = Arr[3];

		stop3.x = Arr[4];
		stop3.y = Arr[5];

		stop4.x = Arr[6];
		stop4.y = Arr[7];

		stop5.x = Arr[8];
		stop5.y = Arr[9];
		fl3=true;
		msgS.data = "sheep";
		cout<<"msgS: " <<msgS << endl;
		fls = true;
	}
	return;
}

bool dist(pointS o)
{
	cout<<"o: " << o.x << " " << o.y<<endl;

	
	float ab1 = sqrt(((o.x-stop1.x)*(o.x-stop1.x))+((o.y-stop1.y)*(o.y-stop1.y)));
cout<<"ab1: " << ab1 <<endl;
	if(ab1 < 1.7) return true;
	float ab2 = sqrt(((o.x-stop2.x)*(o.x-stop2.x))+((o.y-stop2.y)*(o.y-stop2.y)));
cout<<"ab2: " << ab2 <<  endl;
	if(ab2 < 1.7) return true;
	float ab3 = sqrt(((o.x-stop3.x)*(o.x-stop3.x))+((o.y-stop3.y)*(o.y-stop3.y)));
cout<<"ab3: " << ab3 << endl;
	if(ab3 < 1.7) return true;
	float ab4 = sqrt(((o.x-stop4.x)*(o.x-stop4.x))+((o.y-stop4.y)*(o.y-stop4.y)));
cout<<"ab4: " << ab4 <<  endl;
	if(ab4 < 1.7) return true;
	float ab5 = sqrt(((o.x-stop5.x)*(o.x-stop5.x))+((o.y-stop5.y)*(o.y-stop5.y)));
cout<<"ab5: " << ab5 <<endl;
	if(ab5 < 1.7) return true;
	return false;
}

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	if (msg->data == "one") {
		fl = true;
		cout << "fl == true(one)" << endl;
		wolf = "/wolf/pose";
	}
	if (msg->data == "one2") {
		fl = true;
		cout << "fl == true(one2)" << endl;
		wolf = "/wolf2/pose";
	}
	if (msg->data == "one3") {
		fl = true;
		cout << "fl == true(one3)" << endl;
		wolf = "/wolf3/pose";
	}
	else {
		fl2 = true;
	}
}


int main(int argc, char ** argv)
{

	ros::init(argc, argv, "sheep");
	ros::NodeHandle n;
	ros::Publisher pub0 = n.advertise<std_msgs::String>("stopS", 10);
	ros::Publisher pub = n.advertise<std_msgs::String>("wolf/cmd", 10);
	ros::Publisher pub2 = n.advertise<std_msgs::String>("go/cmd", 10);
	ros::Subscriber sub = n.subscribe("sheep/cmd", 10, chatterCallback);
	ros::Subscriber sub3 = n.subscribe("array", 100, arrayCallback);
	string model;
	string buf;
	msgS.data = "xz";

	ros::Rate loop_rate(50);

	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::TransformListener listener;
	tf::StampedTransform tr;

	ros::service::waitForService("gazebo/spawn_sdf_model");
	ros::ServiceClient add_robot =
	    n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	gazebo_msgs::SpawnModel srv;

	ifstream fin("/home/ckobar/.gazebo/models/mars_rover/model.sdf");


	while (!fin.eof() && fin) {
		getline(fin, buf);
		model += buf + "\n";
	}

	srv.request.model_xml = model;
	srv.request.model_name = "sheep";
	geometry_msgs::Pose pose;
	srv.request.initial_pose = pose;
	add_robot.call(srv);

	ros::Publisher pubgaz =
	    n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
	sleep(1.0);

	gazebo_msgs::ModelState msg;
	msg.model_name = "sheep";
	msg.pose.position.x = -2;
	msg.pose.position.y = 2;

	

	pubgaz.publish(msg);

	fl = false;
	come = false;
	int j = 1;
	bool ones = false;
	int check = 0;
	float angle = 4.71;
	bool ans = false;	
	float x = msg.pose.position.x;
	float y = msg.pose.position.y;
	bool wlf = false;
	float alphaT = 0;

	while (ros::ok())
	{	
		
		if(fls){
			pub0.publish(msgS);
			fls = false;
		}
		if(msg.pose.position.x > 10 || msg.pose.position.x < -10 || msg.pose.position.y > 10 || msg.pose.position.y < -10){
			cout<<"sheep wins"<<endl;
			break;
		}
		j ++;
		//angle+=1;
		transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
		tf::Quaternion q;//(tf::Vector3(0, 0, 0), angle);
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "sheep/pose"));
		
		float old_tr_x = fabs(tr.getOrigin().x());
		float old_tr_y = fabs(tr.getOrigin().y());	
			

		if (fl)
		{
			try
			{
				listener.waitForTransform("/sheep/pose", wolf, ros::Time::now(), ros::Duration(0.5));
				listener.lookupTransform("/sheep/pose", wolf, ros::Time(0), tr);
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s", ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}
			if(!wlf){	
				x = tr.getOrigin().x();
				y = tr.getOrigin().y();
				wlf = true;
			}

		
			//if(!ans){
			float old_x = tr.getOrigin().x();
			float old_y = tr.getOrigin().y();	
			
			if(fabs(old_x)<x)
				x = old_x;

			if(fabs(old_y)<y)
				y = old_y;
			
			
			float a = (x);
			float b = (y);
			float alpha = atan(a / b);
			alpha = atan(b / a) - alphaT;
			alphaT = alpha == 0 ? alphaT : alpha;
			bool alT = alpha > 0 ? true : false;
			while ((alpha > 0.05 && alT) || (alpha < -0.05 && !alT))
			{
				float da = 0.01;
				if(alT){
					angle += da;
					alpha -= da;
				}
				else {
					angle -= da;
					alpha += da;
				}
				//cout<<"angle: " <<angle <<"\nalpha: " << alpha <<endl;
				float roll = 0.0; 
				float pitch = 0.0;
				float yaw = angle;
				//setRPY(msg.pose, 0, 0, angle);
				double Yaw = double(yaw) * double(0.5);
				double H = double(pitch) * double(0.5);
				double R = double(roll) * double(0.5);
				double cosYaw = cos(Yaw);
				double sinYaw = sin(Yaw);
				double cosH = cos(H);
				double sinH = sin(H);
				double cosR = cos(R);
				double sinR = sin(R);

				msg.pose.orientation.x = sinR * cosH * cosYaw - cosR * sinH * sinYaw;
				msg.pose.orientation.y = cosR * sinH * cosYaw + sinR * cosH * sinYaw;
				msg.pose.orientation.z = cosR * cosH * sinYaw - sinR * sinH * cosYaw;
				msg.pose.orientation.w = cosR * cosH * cosYaw + sinR * sinH * sinYaw;
				pubgaz.publish(msg);

				loop_rate.sleep();
			}
		
			ans = true;	
			//}

			float dx =  0.01;
			dx *= tr.getOrigin().x() > 0 ? 1 : -1;
			float dy =  0.01;
			dy *= tr.getOrigin().y() > 0 ? 1 : -1;

			cout << "fabs(tr.getOrigin().x()): " << fabs(tr.getOrigin().x()) << "\nfabs(tr.getOrigin().y()): " << fabs(tr.getOrigin().y()) << endl;		
/*
			if (come)
			{
				return 0;
				while ((fabs(msg.pose.position.x) - 6-1) < 0.5 && (fabs(msg.pose.position.y) - 4-1) < 0.5){
				msg.pose.position.x += dx;
					msg.pose.position.y += dy;

					pubgaz.publish(msg);
					loop_rate.sleep();
			}
				return 0;
			}
		*/
			for (int i = 0; i < 100; i++ )
			{

				msg.pose.position.x -= dx;
				msg.pose.position.y -= dy;

				pubgaz.publish(msg);
				loop_rate.sleep();
				
				transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
				tf::Quaternion q;
				q.setRPY(0, 0, 0);
				transform.setRotation(q);
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "sheep/pose"));	
			}
			std_msgs::String msg;
			msg.data = "go";
			pub2.publish(msg);
			fl = false;
			//msg.pose.position.x = -3;
			//msg.pose.position.y = 3;

			//pubgaz.publish(msg);
			//loop_rate.sleep();
			
		}
		
		else
		{
			transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
			tf::Quaternion q;//(tf::Vector3(0, 0, 0), angle);
			q.setRPY(0, 0, 0);
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "sheep/pose"));
			if(fl2)
			{
				std_msgs::String msg;
				msg.data = "step";
				pub2.publish(msg);
				fl2=false;
			}
		ros::spinOnce();
		}
		//loop_rate.sleep();
	}

	return 0;
}

