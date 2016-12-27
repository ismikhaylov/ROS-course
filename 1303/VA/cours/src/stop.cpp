#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <iostream> 
#include "string.h"
#include "std_msgs/Int64.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

using namespace std;

bool fl;
std_msgs::Int32MultiArray dataOut;
int F = 0;
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	if (msg->data == "wolf") {
		F++;
	}
	if (msg->data == "sheep") {
		F++;
	}
	cout<<"F: " << F <<endl;
	return;
	
}

int main(int argc, char ** argv)//20
{
	
	ros::init(argc, argv, "stop");

        ros::NodeHandle n;
	
	fl = false;
	dataOut.data.clear();
	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::TransformListener listener;
	tf::StampedTransform tr;

	ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("array", 10);
	ros::Subscriber sub2 = n.subscribe("stopS", 100, chatterCallback);
	//ros::Publisher pub = n.advertise<std_msgs::String>("two/cmd", 10);
	//ros::Publisher pubF = n.advertise<std_msgs::Int64>("two/cmd", 10);
	ros::Rate loop_rate(50);

        ros::service::waitForService("gazebo/spawn_sdf_model");
	ifstream fin5("/home/ckobar/.gazebo/models/cube_20k/model.sdf");
//40
//for(int i = 0; i<3; i++){

//robot 5
ros::ServiceClient add_robot5 = 
             n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
        gazebo_msgs::SpawnModel srv5;
string model5;
        string buf5;
        while(!fin5.eof()){
            getline(fin5, buf5);
            model5 += buf5 + "\n";
        }

        srv5.request.model_xml = model5;
        srv5.request.model_name = "stop5";
        geometry_msgs::Pose pose5;
        srv5.request.initial_pose = pose5;
        add_robot5.call(srv5);
        //Spawning finished
 
        ros::Publisher pubgaz5 = 
            n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
        sleep(1.0);
	
	gazebo_msgs::ModelState msg5;
	msg5.model_name = "stop5";
	msg5.pose.position.x = (float)-3;
	msg5.pose.position.y = (float)-2;
	pubgaz5.publish(msg5);
	dataOut.data.push_back(-3);
	dataOut.data.push_back(-2);
//robot 4
ifstream fin4("/home/ckobar/.gazebo/models/cube_20k/model.sdf");
ros::ServiceClient add_robot4 = 
             n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
        gazebo_msgs::SpawnModel srv4;
string model4;
        string buf4;
        while(!fin4.eof()){
            getline(fin4, buf4);
            model4 += buf4 + "\n";
        }

        srv4.request.model_xml = model4;
        srv4.request.model_name = "stop4";
        geometry_msgs::Pose pose4;
        srv4.request.initial_pose = pose4;
        add_robot4.call(srv4);
        //Spawning finished
 
        ros::Publisher pubgaz4 = 
            n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
        sleep(1.0);
	
	gazebo_msgs::ModelState msg4;
	msg4.model_name = "stop4";
	msg4.pose.position.x = (float)6;
	msg4.pose.position.y = (float)-3;
	pubgaz4.publish(msg4);
	dataOut.data.push_back(6);
	dataOut.data.push_back(-3);
//robot 3
ifstream fin3("/home/ckobar/.gazebo/models/cube_20k/model.sdf");
ros::ServiceClient add_robot3 = 
             n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
        gazebo_msgs::SpawnModel srv3;
string model3;
        string buf3;
        while(!fin3.eof()){
            getline(fin3, buf3);
            model3 += buf3 + "\n";
        }

        srv3.request.model_xml = model3;
        srv3.request.model_name = "stop3";
        geometry_msgs::Pose pose3;
        srv3.request.initial_pose = pose3;
        add_robot3.call(srv3);
        //Spawning finished
 
        ros::Publisher pubgaz3 = 
            n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
        sleep(1.0);
	
	gazebo_msgs::ModelState msg3;
	msg3.model_name = "stop3";
	msg3.pose.position.x = (float)-2;
	msg3.pose.position.y = (float)5;
	pubgaz3.publish(msg3);
	dataOut.data.push_back(-2);
	dataOut.data.push_back(5);
//robot 2
ifstream fin2("/home/ckobar/.gazebo/models/cube_20k/model.sdf");
ros::ServiceClient add_robot2 = 
             n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
        gazebo_msgs::SpawnModel srv2;
string model2;
        string buf2;
        while(!fin2.eof()){
            getline(fin2, buf2);
            model2 += buf2 + "\n";
        }
	

        srv2.request.model_xml = model2;
        srv2.request.model_name = "stop2";
        geometry_msgs::Pose pose2;
        srv2.request.initial_pose = pose2;
        add_robot2.call(srv2);
        //Spawning finished
 
        ros::Publisher pubgaz2 = 
            n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
        sleep(1.0);
	
	gazebo_msgs::ModelState msg2;
	msg2.model_name = "stop2";
	msg2.pose.position.x = (float)5;
	msg2.pose.position.y = (float)3;
	pubgaz2.publish(msg2);
	dataOut.data.push_back(5);
	dataOut.data.push_back(3);

//robot 1
ifstream fin1("/home/ckobar/.gazebo/models/cube_20k/model.sdf");
ros::ServiceClient add_robot1 = 
             n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
        gazebo_msgs::SpawnModel srv1;
string model1;
        string buf1;
        while(!fin1.eof()){
            getline(fin1, buf1);
            model1 += buf1 + "\n";
        }

        srv1.request.model_xml = model1;
        srv1.request.model_name = "stop1";
        geometry_msgs::Pose pose1;
        srv1.request.initial_pose = pose1;
        add_robot1.call(srv1);
        //Spawning finished
 
        ros::Publisher pubgaz1 = 
            n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
        sleep(1.0);
	
	gazebo_msgs::ModelState msg1;
	msg1.model_name = "stop1";
	msg1.pose.position.x = (float)1;
	msg1.pose.position.y = (float)1;
	pubgaz1.publish(msg1);
	dataOut.data.push_back(1);
	dataOut.data.push_back(1);
	
	bool sto = false;
	int i = 1;
	while (ros::ok())
	{
		if(F==2) sleep(5);
		else {
			pub.publish(dataOut);
			ros::spinOnce();
		}
			
	}

	return 0;
}

