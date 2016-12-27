#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"
#include "std_msgs/Int64.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include <cmath>  

using namespace std;


struct pointS{
	float x;
	float y;
};
bool w1p = false;
bool w2p = false;
bool w3p = false;

int die_sheep = 0;
bool select_path = true;
bool fl;
bool fl3 = false;
bool ans = false;
bool f = false;
int Arr[10];
std_msgs::String msgW;

bool flw = false;

pointS stop1;
pointS stop2;
pointS stop3;
pointS stop4;
pointS stop5;
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
		msgW.data = "wolf";
		cout<<"msgW: " <<msgW<<endl;
		flw = true;
	}

	return;
}

void SheepGo(const std_msgs::String::ConstPtr& msg)
{
	if(msg->data == "go")
		ans = false;
	if(msg->data == "step"){
		fl = false;
		//std_msgs::String msg;
		//msg.data = "step";
		//pub2.publish(msg);
	}
	return;
}




bool dist(pointS o)
{
	cout<<"o: " << o.x << " " << o.y<<endl;

	
	float ab1 = sqrt(((o.x-stop1.x)*(o.x-stop1.x))+((o.y-stop1.y)*(o.y-stop1.y)));
cout<<"ab1: " << ab1 <<endl;
	if(ab1 < 1.6) return true;
	float ab2 = sqrt(((o.x-stop2.x)*(o.x-stop2.x))+((o.y-stop2.y)*(o.y-stop2.y)));
cout<<"ab2: " << ab2 <<  endl;
	if(ab2 < 1.6) return true;
	float ab3 = sqrt(((o.x-stop3.x)*(o.x-stop3.x))+((o.y-stop3.y)*(o.y-stop3.y)));
cout<<"ab3: " << ab3 << endl;
	if(ab3 < 1.6) return true;
	float ab4 = sqrt(((o.x-stop4.x)*(o.x-stop4.x))+((o.y-stop4.y)*(o.y-stop4.y)));
cout<<"ab4: " << ab4 <<  endl;
	if(ab4 < 1.6) return true;
	float ab5 = sqrt(((o.x-stop5.x)*(o.x-stop5.x))+((o.y-stop5.y)*(o.y-stop5.y)));
cout<<"ab5: " << ab5 <<endl;
	if(ab5 < 1.6) return true;
	return false;
}

pointS distStep(pointS* o, pointS stop)
{
	float ab = 0;pointS fin;
cout<<"ab: " << ab <<endl;cout<<"fin: " << fin.x << " " << fin.y << endl;
	
	float ab1 = sqrt(((o[0].x-stop.x)*(o[0].x-stop.x))+((o[0].y-stop.y)*(o[0].y-stop.y)));
	ab = ab1;
	fin = o[0];
cout<<"ab: " << ab <<endl;cout<<"fin: " << fin.x << " " << fin.y << endl;
	//if(ab1 < 1.5) return true;
	float ab2 = sqrt(((o[1].x-stop.x)*(o[1].x-stop.x))+((o[1].y-stop.y)*(o[1].y-stop.y)));
	if(ab>ab2){
		ab = ab2;
		fin = o[1];
	}
cout<<"ab: " << ab <<endl;cout<<"fin: " << fin.x << " " << fin.y << endl;
	//if(ab2 < 1.5) return true;
	float ab3 = sqrt(((o[2].x-stop.x)*(o[2].x-stop.x))+((o[2].y-stop.y)*(o[2].y-stop.y)));
	if(ab>ab3){
		ab = ab3;
		fin = o[2];
	}
cout<<"ab: " << ab <<endl;cout<<"fin: " << fin.x << " " << fin.y << endl;
	//if(ab3 < 1.5) return true;
	float ab4 = sqrt(((o[3].x-stop.x)*(o[3].x-stop.x))+((o[3].y-stop.y)*(o[3].y-stop.y)));
	if(ab>ab4){
		ab = ab4;
		fin = o[3];
	}
cout<<"ab: " << ab <<endl;
cout<<"fin: " << fin.x << " " << fin.y << endl;	
	return fin;
}

pointS distW(pointS stop){
	float ab = 0;pointS fin;
	
	float ab1 = sqrt(((4-stop.x)*(4-stop.x))+((2-stop.y)*(2-stop.y)));
	ab = ab1;
	fin.x = 4;
	fin.y = 2;
//cout<<"ab: " << ab <<endl;cout<<"fin: " << fin.x << " " << fin.y << endl;
	//if(ab1 < 1.5) return true;
	float ab2 = sqrt(((0-stop.x)*(0-stop.x))+((2-stop.y)*(2-stop.y)));
	if(ab>ab2){
		ab = ab2;
		fin.x = 0;
		fin.y = 2;
	}
//cout<<"ab: " << ab <<endl;cout<<"fin: " << fin.x << " " << fin.y << endl;
	//if(ab2 < 1.5) return true;
	float ab3 = sqrt(((2-stop.x)*(2-stop.x))+((0-stop.y)*(0-stop.y)));
	if(ab>ab3){
		ab = ab3;
		fin.x = 2;
		fin.y = 0;
	}
//cout<<"ab: " << ab <<endl;cout<<"fin: " << fin.x << " " << fin.y << endl;
	//if(ab3 < 1.5) return true;
	float ab4 = sqrt(((2-stop.x)*(2-stop.x))+((4-stop.y)*(4-stop.y)));
	if(ab>ab4){
		ab = ab4;
		fin.x = 2;
		fin.y = 4;
	}
//cout<<"ab: " << ab <<endl;
//cout<<"fin: " << fin.x << " " << fin.y << endl;	
	return fin;
	
}

int main(int argc, char ** argv)//20
{
	ros::init(argc, argv, "wolf");

        ros::NodeHandle n;

	
	fl = true;
	msgW.data = "xz";

	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::TransformListener listener;
	tf::StampedTransform tr;

	tf::TransformBroadcaster br2;
	tf::Transform transform2;
	tf::TransformListener listener2;
	tf::StampedTransform tr2;
	
	tf::TransformBroadcaster br3;
	tf::Transform transform3;
	tf::TransformListener listener3;
	tf::StampedTransform tr3;

	ros::Publisher pub = n.advertise<std_msgs::String>("sheep/cmd", 10);
	ros::Publisher pub0 = n.advertise<std_msgs::String>("stopS", 10);
	ros::Subscriber sub2 = n.subscribe("go/cmd", 100, SheepGo);
	ros::Subscriber sub3 = n.subscribe("array", 100, arrayCallback);
	//ros::Publisher pub2 = n.advertise<std_msgs::String>("wolf2/cmd", 10);
	//ros::Publisher pubF = n.advertise<std_msgs::Int64>("two/cmd", 10);
	ros::Rate loop_rate(50);

	

        ros::service::waitForService("gazebo/spawn_sdf_model");

  //wolf 3
     ros::ServiceClient add_robot3 = 
             n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
        gazebo_msgs::SpawnModel srv3;
        ifstream fin3("/home/ckobar/.gazebo/models/pioneer3at/model.sdf");
        string model3;
        string buf3;
        while(!fin3.eof()){
            getline(fin3, buf3);
            model3 += buf3 + "\n";
        }
        srv3.request.model_xml = model3;
        srv3.request.model_name = "wolf3";
        geometry_msgs::Pose pose3;
        srv3.request.initial_pose = pose3;
        add_robot3.call(srv3);
        //Spawning finished
//60
        ros::Publisher pubgaz3 = 
            n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
        sleep(1.0);
        gazebo_msgs::ModelState msg3;
        msg3.model_name = "wolf3";
        msg3.pose.position.x = 4.5;
	msg3.pose.position.y = 4.5;
        pubgaz3.publish(msg3);
//wolf 2
     ros::ServiceClient add_robot2 = 
             n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
        gazebo_msgs::SpawnModel srv2;
        ifstream fin2("/home/ckobar/.gazebo/models/pioneer3at/model.sdf");
        string model2;
        string buf2;
        while(!fin2.eof()){
            getline(fin2, buf2);
            model2 += buf2 + "\n";
        }
        srv2.request.model_xml = model2;
        srv2.request.model_name = "wolf2";
        geometry_msgs::Pose pose2;
        srv2.request.initial_pose = pose2;
        add_robot2.call(srv2);
        //Spawning finished
//60
        ros::Publisher pubgaz2 = 
            n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
        sleep(1.0);
        gazebo_msgs::ModelState msg2;
        msg2.model_name = "wolf2";
        msg2.pose.position.x = -5;
	msg2.pose.position.y = 5;
        pubgaz2.publish(msg2);

//wolf 1
     ros::ServiceClient add_robot = 
             n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
        gazebo_msgs::SpawnModel srv;
        ifstream fin("/home/ckobar/.gazebo/models/pioneer3at/model.sdf");
        string model;
        string buf;
        while(!fin.eof()){
            getline(fin, buf);
            model += buf + "\n";
        }
        srv.request.model_xml = model;
        srv.request.model_name = "wolf";
        geometry_msgs::Pose pose;
        srv.request.initial_pose = pose;
        add_robot.call(srv);
        //Spawning finished
//60
        ros::Publisher pubgaz = 
            n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
        sleep(1.0);
        gazebo_msgs::ModelState msg;
        msg.model_name = "wolf";
        msg.pose.position.x = -4.5;
	msg.pose.position.y = -4.5;
        pubgaz.publish(msg);

	float start_x = msg.pose.position.x;
	float start_y = msg.pose.position.y;
		
	float angle = 3.14;
	float angle2 = 3.14;
	float angle3 = 3.14;
	//bool ans2 = false;
	float alphaT1 = 0;
	float alphaT2 = 0;
	float alphaT3 = 0;

	//sleep(5);
	pointS wl1;
	wl1.x = -4.5;
	wl1.y = -4.5;
	pointS wl2;
	wl2.x = -5;
	wl2.y = 5;
	pointS wl3;
	wl3.x = 4.5;
	wl3.y = 4.5;
	pointS dwl1 = distW(wl1);
	pointS dwl2 = distW(wl2);
	pointS dwl3 = distW(wl3);



	while (ros::ok())
	{
		if(flw){
			pub0.publish(msgW);
			flw = false;
		}
		if(die_sheep>=3) {
			cout<<"woolf wins"<<endl;
			break;
		}
		
		//if (pub.getNumSubscribers() == 0)
		//{			
		//	continue;
		//}
		//cout << "zero if" << endl;
		
		//cout << "one if" << endl;

		
		transform2.setOrigin( tf::Vector3(msg2.pose.position.x, msg2.pose.position.y, 0.0) );
		tf::Quaternion q2;
		q2.setRPY(0, 0, 0);
		transform2.setRotation(q2);
		br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "world", "wolf2/pose"));
		
		transform3.setOrigin( tf::Vector3(msg3.pose.position.x, msg3.pose.position.y, 0.0) );
		tf::Quaternion q3;
		q3.setRPY(0, 0, 0);
		transform3.setRotation(q3);
		br3.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "world", "wolf3/pose"));

		transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "wolf/pose"));

if(select_path){
		if (!fl)
		{
			//cout << "two if" << endl;
			try
			{
			listener.waitForTransform("/wolf/pose", "/sheep/pose", ros::Time::now(), ros::Duration(0.5));
			listener.lookupTransform("/wolf/pose", "/sheep/pose", ros::Time(0), tr);
			}
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s", ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}
			if(!ans){
			
			float x = tr.getOrigin().x();
			float y = tr.getOrigin().y();		
			cout<<"x: "<<x<<" y "<<y<<endl;
			float a = (x);
			float b = (y);
			float alpha = atan(a / b);
			alpha = atan(b / a) - alphaT1;
			alphaT1 = alpha == 0 ? alphaT1 : alpha;
			cout<<"alpha one: "<<alpha<<endl;
			cout<<"angle one: "<<angle<<endl;
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
			}
			listener2.waitForTransform("/wolf2/pose", "/sheep/pose", ros::Time::now(), ros::Duration(0.6));
			listener2.lookupTransform("/wolf2/pose", "/sheep/pose", ros::Time(0), tr2);
			
			if(!ans){
			
			float x2 = tr2.getOrigin().x();
			float y2 = tr2.getOrigin().y();		
			cout<<"x2: "<<x2<<" y2 "<<y2<<endl;
			float a2 = (x2);
			float b2 = (y2);
			float alpha2 = atan(a2 / b2);
			alpha2 = atan(b2 / a2) - alphaT2;
			alphaT2 = alpha2 == 0 ? alphaT2 : alpha2;
			cout<<"alpha two: "<<alpha2<<endl;
			cout<<"angle two: "<<angle2<<endl;
			bool alT2 = alpha2 > 0 ? true : false;
			while ((alpha2 > 0.05 && alT2) || (alpha2 < -0.05 && !alT2))
			{
				float da = 0.01;
				if(alT2){
					angle2 += da;
					alpha2 -= da;
				}
				else {
					angle2 -= da;
					alpha2 += da;
				}
				
				//cout<<"angle: " <<angle <<"\nalpha: " << alpha <<endl;
				float roll = 0.0; 
				float pitch = 0.0;
				float yaw = angle2;
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

				msg2.pose.orientation.x = sinR * cosH * cosYaw - cosR * sinH * sinYaw;
				msg2.pose.orientation.y = cosR * sinH * cosYaw + sinR * cosH * sinYaw;
				msg2.pose.orientation.z = cosR * cosH * sinYaw - sinR * sinH * cosYaw;
				msg2.pose.orientation.w = cosR * cosH * cosYaw + sinR * sinH * sinYaw;
				pubgaz2.publish(msg2);

				loop_rate.sleep();
			}
			}
			listener3.waitForTransform("/wolf3/pose", "/sheep/pose", ros::Time::now(), ros::Duration(0.4));
			listener3.lookupTransform("/wolf3/pose", "/sheep/pose", ros::Time(0), tr3);
			
			if(!ans){
			
			float x3 = tr3.getOrigin().x();
			float y3 = tr3.getOrigin().y();		
			cout<<"x3: "<<x3<<" y3 "<<y3<<endl;
			float a3 = (x3);
			float b3 = (y3);
			float alpha3 = atan(a3 / b3);
			alpha3 = atan(b3 / a3) - alphaT3;
			alphaT3 = alpha3 == 0 ? alphaT3 : alpha3;
			cout<<"alpha 3: "<<alpha3<<endl;
			cout<<"angle 3: "<<angle3<<endl;
			bool alT3 = alpha3 > 0 ? true : false;
			while ((alpha3 > 0.05 && alT3) || (alpha3 < -0.05 && !alT3))
			{
				float da = 0.01;
				if(alT3){
					angle3 += da;
					alpha3 -= da;
				}
				else {
					angle3 -= da;
					alpha3 += da;
				}
				//cout<<"angle: " <<angle <<"\nalpha: " << alpha <<endl;
				float roll = 0.0; 
				float pitch = 0.0;
				float yaw = angle3;
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

				msg3.pose.orientation.x = sinR * cosH * cosYaw - cosR * sinH * sinYaw;
				msg3.pose.orientation.y = cosR * sinH * cosYaw + sinR * cosH * sinYaw;
				msg3.pose.orientation.z = cosR * cosH * sinYaw - sinR * sinH * cosYaw;
				msg3.pose.orientation.w = cosR * cosH * cosYaw + sinR * sinH * sinYaw;
				pubgaz3.publish(msg3);

				loop_rate.sleep();
			}		
			ans = true;
			}
			std_msgs::String msgA;
			msgA.data = "step";
			
			if (fabs(tr.getOrigin().x()) < 3 && fabs(tr.getOrigin().y()) < 3)
			{
				msgA.data = "one";
				cout << "msgA: " << msgA << endl;
			}

			float dx = 0.01;
			dx *= tr.getOrigin().x() > 0 ? 1 : -1;
			float dy = 0.01;
			dy *= tr.getOrigin().y() > 0 ? 1 : -1;

			pointS w1;
			w1.x = msg.pose.position.x+(dx*100);
			w1.y = msg.pose.position.y+(dy*100);
			cout<<"one dist"<<endl;
			if(!dist(w1)){
				for (int i = 0; i < 100; i++ ) {
					msg.pose.position.x += dx;
					msg.pose.position.y += dy;

					pubgaz.publish(msg);
					transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
					tf::Quaternion q;
					q.setRPY(0, 0, 0);
					transform.setRotation(q);
					br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "wolf/pose"));

					loop_rate.sleep();

				}
			}
			if(fabs(tr.getOrigin().x()) < 1 && fabs(tr.getOrigin().y()) < 1) die_sheep++;
			else{
				pointS t;
				t.x = tr.getOrigin().x() + msg.pose.position.x;
				t.y = tr.getOrigin().y() + msg.pose.position.y;
//cout << "msg.pose.position.x: " << msg.pose.position.x <<" " <<msg.pose.position.y << endl;
				pointS temp1;
				pointS temp2;
				pointS temp3;
				pointS temp4;
				pointS arP[4];
				int iT = 0;
				pointS tempF;
				tempF.x= 100;
				tempF.y= 100;

				temp1.x = msg.pose.position.x+1.0;
				temp1.y = msg.pose.position.y;
//cout << "temp1: " << temp1.x <<" " <<temp1.y << endl;

				temp2.x = msg.pose.position.x-1.0;
				temp2.y = msg.pose.position.y;
//cout << "temp2: " << temp2.x <<" " <<temp2.y << endl;

				temp3.x = msg.pose.position.x;
				temp3.y = msg.pose.position.y+1.0;
//cout << "temp31: " << temp3.x <<" " <<temp3.y << endl;

				temp4.x = msg.pose.position.x;
				temp4.y = msg.pose.position.y-1.0;
//cout << "temp4: " << temp4.x <<" " <<temp4.y << endl;

				if(!dist(temp1)) {
					arP[iT] = temp1;
					iT++;
				}
				if(!dist(temp2)) {
					arP[iT] = temp2;
					iT++;
				}
				if(!dist(temp3)) {
					arP[iT] = temp3;
					iT++;
				}
				if(!dist(temp4)) {
					arP[iT] = temp4;
					iT++;
				}
				while(iT<4)
				{
					arP[iT] = tempF;
					iT++;
				}
				cout<<"one ptr"<<endl;
				pointS temp = distStep(arP, t);

//-----------------

				float xT1 = temp.x-msg.pose.position.x;
				float yT1 = temp.y-msg.pose.position.y;		
				//cout<<"x3: "<<x3<<" y3 "<<y3<<endl;
				float aT1 = (xT1);
				float bT1 = (yT1);
				float alpha1 = atan(aT1 / bT1);
				alpha1 = atan(bT1 / aT1);// - alphaT3;
				
				bool alT1 = alpha1 > 0 ? true : false;
				float angAlf = angle;
				bool alT = alpha1 > 0 ? true : false;
				while ((alpha1 > 0.05 && alT1) || (alpha1 < -0.05 && !alT1))
				{
					float da = 0.01;
					if(alT1){
						angle += da;
						alpha1 -= da;
					}
					else {
						angle -= da;
						alpha1 += da;
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
				
				
//---------------
				
				float dx = 0.01;
				dx *= (temp.x-msg.pose.position.x) > 0 ? 1 : -1;
				float dy = 0.01;
				dy *= (temp.y-msg.pose.position.y) > 0 ? 1 : -1;
				for (int i = 0; i < 100; i++ ) {
					msg.pose.position.x += dx;
					msg.pose.position.y += dy;

					pubgaz.publish(msg);
					transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
					tf::Quaternion q;
					q.setRPY(0, 0, 0);
					transform.setRotation(q);
					br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "wolf/pose"));

					loop_rate.sleep();

				}
				
//-------------
				while (fabs(angAlf-angle)>0.05)
				{
					float da = 0.01;
					if(alT){
						angle -= da;
						//alpha -= da;
					}
					else {
						angle += da;
						//alpha += da;
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
//--------------
				
			}
			if (fabs(tr2.getOrigin().x()) < 3 && fabs(tr2.getOrigin().y()) < 3)
			{
				msgA.data = "one2";
				cout << "msgA2: " << msgA << endl;
			}

			float dx2 =  0.01;
			dx2 *= tr2.getOrigin().x() > 0 ? 1 : -1;
			float dy2 =  0.01;
			dy2 *= tr2.getOrigin().y() > 0 ? 1 : -1;

			pointS w2;
			w2.x = msg2.pose.position.x+(dx2*100);
			w2.y = msg2.pose.position.y+(dy2*100);
			cout<<"two dist"<<endl;
			if(!dist(w2)){
			for (int i = 0; i < 100; i++ ) {
				msg2.pose.position.x += dx2;
				msg2.pose.position.y += dy2;

				pubgaz2.publish(msg2);
				transform2.setOrigin( tf::Vector3(msg2.pose.position.x, msg2.pose.position.y, 0.0) );
				tf::Quaternion q2;
				q2.setRPY(0, 0, 0);
				transform2.setRotation(q2);
				br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "world", "wolf2/pose"));
				loop_rate.sleep();

			}
			}
			if(fabs(tr2.getOrigin().x()) < 1 && fabs(tr2.getOrigin().y()) < 1) die_sheep++;
			else{
				pointS t;
				t.x = tr2.getOrigin().x() + msg2.pose.position.x;
				t.y = tr2.getOrigin().y() + msg2.pose.position.y;
				pointS temp1;
				pointS temp2;
				pointS temp3;
				pointS temp4;
				pointS arP[4];
				int iT = 0;
				pointS tempF;
				tempF.x= 100;
				tempF.y= 100;

				temp1.x = msg2.pose.position.x+1;
				temp1.y = msg2.pose.position.y;

				temp2.x = msg2.pose.position.x-1;
				temp2.y = msg2.pose.position.y;

				temp3.x = msg2.pose.position.x;
				temp3.y = msg2.pose.position.y+1;

				temp4.x = msg2.pose.position.x;
				temp4.y = msg2.pose.position.y-1;

				if(!dist(temp1)) {
					arP[iT] = temp1;
					iT++;
				}
				if(!dist(temp2)) {
					arP[iT] = temp2;
					iT++;
				}
				if(!dist(temp3)) {
					arP[iT] = temp3;
					iT++;
				}
				if(!dist(temp4)) {
					arP[iT] = temp4;
					iT++;
				}
				while(iT<4)
				{
					arP[iT] = tempF;
					iT++;
				}
				cout<<"two ptr"<<endl;
				pointS temp = distStep(arP, t);

//-----------------

				float xT1 = temp.x-msg2.pose.position.x;
				float yT1 = temp.y-msg2.pose.position.y;		
				//cout<<"x3: "<<x3<<" y3 "<<y3<<endl;
				float aT1 = (xT1);
				float bT1 = (yT1);
				float alpha1 = atan(aT1 / bT1);
				alpha1 = atan(bT1 / aT1);// - alphaT3;
				
				bool alT1 = alpha1 > 0 ? true : false;
				float angAlf = angle2;
				bool alT = alpha1 > 0 ? true : false;
				while ((alpha1 > 0.05 && alT1) || (alpha1 < -0.05 && !alT1))
				{
					float da = 0.01;
					if(alT1){
						angle2 += da;
						alpha1 -= da;
					}
					else {
						angle2 -= da;
						alpha1 += da;
					}
					//cout<<"angle: " <<angle <<"\nalpha: " << alpha <<endl;
					float roll = 0.0; 
					float pitch = 0.0;
					float yaw = angle2;
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

					msg2.pose.orientation.x = sinR * cosH * cosYaw - cosR * sinH * sinYaw;
					msg2.pose.orientation.y = cosR * sinH * cosYaw + sinR * cosH * sinYaw;
					msg2.pose.orientation.z = cosR * cosH * sinYaw - sinR * sinH * cosYaw;
					msg2.pose.orientation.w = cosR * cosH * cosYaw + sinR * sinH * sinYaw;
					pubgaz2.publish(msg2);

					loop_rate.sleep();
				}	
				
				
//---------------
				float dx = 0.01;
				dx *= (temp.x-msg2.pose.position.x) > 0 ? 1 : -1;
				float dy = 0.01;
				dy *= (temp.y-msg2.pose.position.y) > 0 ? 1 : -1;

				for (int i = 0; i < 100; i++ ) {
					msg2.pose.position.x += dx;
					msg2.pose.position.y += dy;

					pubgaz2.publish(msg2);
					transform2.setOrigin( tf::Vector3(msg2.pose.position.x, msg2.pose.position.y, 0.0) );
					tf::Quaternion q2;
					q2.setRPY(0, 0, 0);
					transform2.setRotation(q2);
					br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "world", "wolf2/pose"));
					loop_rate.sleep();


				}
//-------------
				while (fabs(angAlf-angle2)>0.05)
				{
					float da = 0.01;
					if(alT){
						angle2 -= da;
						//alpha -= da;
					}
					else {
						angle2 += da;
						//alpha += da;
					}
				
					//cout<<"angle: " <<angle <<"\nalpha: " << alpha <<endl;
					float roll = 0.0; 
					float pitch = 0.0;
					float yaw = angle2;
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

					msg2.pose.orientation.x = sinR * cosH * cosYaw - cosR * sinH * sinYaw;
					msg2.pose.orientation.y = cosR * sinH * cosYaw + sinR * cosH * sinYaw;
					msg2.pose.orientation.z = cosR * cosH * sinYaw - sinR * sinH * cosYaw;
					msg2.pose.orientation.w = cosR * cosH * cosYaw + sinR * sinH * sinYaw;
					pubgaz2.publish(msg2);

					loop_rate.sleep();
				}
//--------------
				
			}
			
			if (fabs(tr3.getOrigin().x()) < 3 && fabs(tr3.getOrigin().y()) < 3)
			{
				msgA.data = "one3";
				cout << "msgA3: " << msgA << endl;
			}

			float dx3 =  0.01;
			dx3 *= tr3.getOrigin().x() > 0 ? 1 : -1;
			float dy3 =  0.01;
			dy3 *= tr3.getOrigin().y() > 0 ? 1 : -1;
			
			pointS w3;
			w3.x = msg3.pose.position.x+(dx3*100);
			w3.y = msg3.pose.position.y+(dy3*100);
			cout<<"three dist"<<endl;
			if(!dist(w3)){
			for (int i = 0; i < 100; i++ ) {
				msg3.pose.position.x += dx3;
				msg3.pose.position.y += dy3;

				pubgaz3.publish(msg3);
				transform3.setOrigin( tf::Vector3(msg3.pose.position.x, msg3.pose.position.y, 0.0) );
				tf::Quaternion q3;
				q3.setRPY(0, 0, 0);
				transform3.setRotation(q3);
				br3.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "world", "wolf3/pose"));

				loop_rate.sleep();

			}
			}
			if(fabs(tr3.getOrigin().x()) < 1 && fabs(tr3.getOrigin().y()) < 1) die_sheep++;
			else{
				pointS t;
				t.x = tr3.getOrigin().x() + msg3.pose.position.x;
				t.y = tr3.getOrigin().y() + msg3.pose.position.y;
				pointS temp1;
				pointS temp2;
				pointS temp3;
				pointS temp4;
				pointS arP[4];
				int iT = 0;
				pointS tempF;
				tempF.x= 100;
				tempF.y= 100;
				temp1.x = msg3.pose.position.x+1;
				temp1.y = msg3.pose.position.y;

				temp2.x = msg3.pose.position.x-1;
				temp2.y = msg3.pose.position.y;

				temp3.x = msg3.pose.position.x;
				temp3.y = msg3.pose.position.y+1;

				temp4.x = msg3.pose.position.x;
				temp4.y = msg3.pose.position.y-1;

				if(!dist(temp1)) {
					arP[iT] = temp1;
					iT++;
				}
				if(!dist(temp2)) {
					arP[iT] = temp2;
					iT++;
				}
				if(!dist(temp3)) {
					arP[iT] = temp3;
					iT++;
				}
				if(!dist(temp4)) {
					arP[iT] = temp4;
					iT++;
				}
				while(iT<4)
				{
					arP[iT] = tempF;
					iT++;
				}
				cout<<"three ptr"<<endl;
				pointS temp = distStep(arP, t);
//-----------------

				float xT1 = temp.x-msg3.pose.position.x;
				float yT1 = temp.y-msg3.pose.position.y;		
				//cout<<"x3: "<<x3<<" y3 "<<y3<<endl;
				float aT1 = (xT1);
				float bT1 = (yT1);
				float alpha1 = atan(aT1 / bT1);
				alpha1 = atan(bT1 / aT1);// - alphaT3;
				
				bool alT1 = alpha1 > 0 ? true : false;
				float angAlf = angle3;
				bool alT = alpha1 > 0 ? true : false;
				while ((alpha1 > 0.05 && alT1) || (alpha1 < -0.05 && !alT1))
				{
					float da = 0.01;
					if(alT1){
						angle3 += da;
						alpha1 -= da;
					}
					else {
						angle3 -= da;
						alpha1 += da;
					}
					//cout<<"angle: " <<angle <<"\nalpha: " << alpha <<endl;
					float roll = 0.0; 
					float pitch = 0.0;
					float yaw = angle3;
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

					msg3.pose.orientation.x = sinR * cosH * cosYaw - cosR * sinH * sinYaw;
					msg3.pose.orientation.y = cosR * sinH * cosYaw + sinR * cosH * sinYaw;
					msg3.pose.orientation.z = cosR * cosH * sinYaw - sinR * sinH * cosYaw;
					msg3.pose.orientation.w = cosR * cosH * cosYaw + sinR * sinH * sinYaw;
					pubgaz3.publish(msg3);

					loop_rate.sleep();
				}	
				
				
//---------------

				float dx = 0.01;
				dx *= (temp.x-msg3.pose.position.x) > 0 ? 1 : -1;
				float dy = 0.01;
				dy *= (temp.y-msg3.pose.position.y) > 0 ? 1 : -1;

				for (int i = 0; i < 100; i++ ) {
					msg3.pose.position.x += dx;
					msg3.pose.position.y += dy;

					pubgaz3.publish(msg3);
					transform3.setOrigin( tf::Vector3(msg3.pose.position.x, msg3.pose.position.y, 0.0) );
					tf::Quaternion q3;
					q3.setRPY(0, 0, 0);
					transform3.setRotation(q3);
					br3.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "world", "wolf3/pose"));

					loop_rate.sleep();

				}
//-------------
				while (fabs(angAlf-angle3)>0.05)
				{
					float da = 0.01;
					if(alT){
						angle3 -= da;
						//alpha -= da;
					}
					else {
						angle3 += da;
						//alpha += da;
					}
				
					//cout<<"angle: " <<angle <<"\nalpha: " << alpha <<endl;
					float roll = 0.0; 
					float pitch = 0.0;
					float yaw = angle3;
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

					msg3.pose.orientation.x = sinR * cosH * cosYaw - cosR * sinH * sinYaw;
					msg3.pose.orientation.y = cosR * sinH * cosYaw + sinR * cosH * sinYaw;
					msg3.pose.orientation.z = cosR * cosH * sinYaw - sinR * sinH * cosYaw;
					msg3.pose.orientation.w = cosR * cosH * cosYaw + sinR * sinH * sinYaw;
					pubgaz3.publish(msg3);

					loop_rate.sleep();
				}
//--------------
				
			}
			
			pub.publish(msgA);
			
			fl = true;
		loop_rate.sleep();
		}
		
		else
		{
			if(!f){
				fl = false;
				f = true;
			}
			ros::spinOnce();
		}

		ros::spinOnce();
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
else{
while(!w1p && !w2p && w3p){
		if (!fl)
		{
			
			if(!ans){
			if(!w1p){
			
			float x = dwl1.x-msg.pose.position.x;
			float y = dwl1.y-msg.pose.position.y;		
			//cout<<"x: "<<x<<" y "<<y<<endl;
			float a = (x);
			float b = (y);
			float alpha = atan(a / b);
			alpha = atan(b / a) - alphaT1;
			alphaT1 = alpha == 0 ? alphaT1 : alpha;
			cout<<"alpha one: "<<alpha<<endl;
			cout<<"angle one: "<<angle<<endl;
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
			}
			}
			
			if(!ans){
			if(!w2p){
			float x2 = dwl2.x-msg2.pose.position.x;
			float y2 = dwl2.y-msg2.pose.position.y;		
			//cout<<"x2: "<<x2<<" y2 "<<y2<<endl;
			float a2 = (x2);
			float b2 = (y2);
			float alpha2 = atan(a2 / b2);
			alpha2 = atan(b2 / a2) - alphaT2;
			alphaT2 = alpha2 == 0 ? alphaT2 : alpha2;
			cout<<"alpha two: "<<alpha2<<endl;
			cout<<"angle two: "<<angle2<<endl;
			bool alT2 = alpha2 > 0 ? true : false;
			while ((alpha2 > 0.05 && alT2) || (alpha2 < -0.05 && !alT2))
			{
				float da = 0.01;
				if(alT2){
					angle2 += da;
					alpha2 -= da;
				}
				else {
					angle2 -= da;
					alpha2 += da;
				}
				
				//cout<<"angle: " <<angle <<"\nalpha: " << alpha <<endl;
				float roll = 0.0; 
				float pitch = 0.0;
				float yaw = angle2;
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

				msg2.pose.orientation.x = sinR * cosH * cosYaw - cosR * sinH * sinYaw;
				msg2.pose.orientation.y = cosR * sinH * cosYaw + sinR * cosH * sinYaw;
				msg2.pose.orientation.z = cosR * cosH * sinYaw - sinR * sinH * cosYaw;
				msg2.pose.orientation.w = cosR * cosH * cosYaw + sinR * sinH * sinYaw;
				pubgaz2.publish(msg2);

				loop_rate.sleep();
			}
			}
			}
			if(!ans){
			if(!w3p){
			float x3 = dwl3.x-msg3.pose.position.x;
			float y3 = dwl3.y-msg3.pose.position.y;	
			//cout<<"x3: "<<x3<<" y3 "<<y3<<endl;
			float a3 = (x3);
			float b3 = (y3);
			float alpha3 = atan(a3 / b3);
			alpha3 = atan(b3 / a3) - alphaT3;
			alphaT3 = alpha3 == 0 ? alphaT3 : alpha3;
			cout<<"alpha 3: "<<alpha3<<endl;
			cout<<"angle 3: "<<angle3<<endl;
			bool alT3 = alpha3 > 0 ? true : false;
			while ((alpha3 > 0.05 && alT3) || (alpha3 < -0.05 && !alT3))
			{
				float da = 0.01;
				if(alT3){
					angle3 += da;
					alpha3 -= da;
				}
				else {
					angle3 -= da;
					alpha3 += da;
				}
				//cout<<"angle: " <<angle <<"\nalpha: " << alpha <<endl;
				float roll = 0.0; 
				float pitch = 0.0;
				float yaw = angle3;
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

				msg3.pose.orientation.x = sinR * cosH * cosYaw - cosR * sinH * sinYaw;
				msg3.pose.orientation.y = cosR * sinH * cosYaw + sinR * cosH * sinYaw;
				msg3.pose.orientation.z = cosR * cosH * sinYaw - sinR * sinH * cosYaw;
				msg3.pose.orientation.w = cosR * cosH * cosYaw + sinR * sinH * sinYaw;
				pubgaz3.publish(msg3);

				loop_rate.sleep();
			}		
			ans = true;
			}
			}
			std_msgs::String msgA;
			msgA.data = "step";
			
			//if (fabs(tr.getOrigin().x()) < 3 && fabs(tr.getOrigin().y()) < 3)
			//{
			//	msgA.data = "one";
			//	cout << "msgA: " << msgA << endl;
			//}
			if(!w1p){
			float dx = 0.01;
			dx *= dwl1.x-msg.pose.position.x > 0 ? 1 : -1;
			float dy = 0.01;
			dy *= dwl1.y-msg.pose.position.y > 0 ? 1 : -1;

			pointS w1;
			w1.x = msg.pose.position.x+(dx*100);
			w1.y = msg.pose.position.y+(dy*100);
			cout<<"one dist"<<endl;
			if(!dist(w1)){
				for (int i = 0; i < 100; i++ ) {
					msg.pose.position.x += dx;
					msg.pose.position.y += dy;

					pubgaz.publish(msg);
					transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
					tf::Quaternion q;
					q.setRPY(0, 0, 0);
					transform.setRotation(q);
					br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "wolf/pose"));

					loop_rate.sleep();

				}
			}
			if(fabs(tr.getOrigin().x()) < 1 && fabs(tr.getOrigin().y()) < 1) die_sheep++;
			else{
				pointS t;
				t.x = tr.getOrigin().x() + msg.pose.position.x;
				t.y = tr.getOrigin().y() + msg.pose.position.y;
//cout << "msg.pose.position.x: " << msg.pose.position.x <<" " <<msg.pose.position.y << endl;
				pointS temp1;
				pointS temp2;
				pointS temp3;
				pointS temp4;
				pointS arP[4];
				int iT = 0;
				pointS tempF;
				tempF.x= 100;
				tempF.y= 100;

				temp1.x = msg.pose.position.x+1.0;
				temp1.y = msg.pose.position.y;
//cout << "temp1: " << temp1.x <<" " <<temp1.y << endl;

				temp2.x = msg.pose.position.x-1.0;
				temp2.y = msg.pose.position.y;
//cout << "temp2: " << temp2.x <<" " <<temp2.y << endl;

				temp3.x = msg.pose.position.x;
				temp3.y = msg.pose.position.y+1.0;
//cout << "temp31: " << temp3.x <<" " <<temp3.y << endl;

				temp4.x = msg.pose.position.x;
				temp4.y = msg.pose.position.y-1.0;
//cout << "temp4: " << temp4.x <<" " <<temp4.y << endl;

				if(!dist(temp1)) {
					arP[iT] = temp1;
					iT++;
				}
				if(!dist(temp2)) {
					arP[iT] = temp2;
					iT++;
				}
				if(!dist(temp3)) {
					arP[iT] = temp3;
					iT++;
				}
				if(!dist(temp4)) {
					arP[iT] = temp4;
					iT++;
				}
				while(iT<4)
				{
					arP[iT] = tempF;
					iT++;
				}
				cout<<"one ptr"<<endl;
				pointS temp = distStep(arP, t);

//-----------------

				float xT1 = temp.x-msg.pose.position.x;
				float yT1 = temp.y-msg.pose.position.y;		
				//cout<<"x3: "<<x3<<" y3 "<<y3<<endl;
				float aT1 = (xT1);
				float bT1 = (yT1);
				float alpha1 = atan(aT1 / bT1);
				alpha1 = atan(bT1 / aT1);// - alphaT3;
				
				bool alT1 = alpha1 > 0 ? true : false;
				float angAlf = angle;
				bool alT = alpha1 > 0 ? true : false;
				while ((alpha1 > 0.05 && alT1) || (alpha1 < -0.05 && !alT1))
				{
					float da = 0.01;
					if(alT1){
						angle += da;
						alpha1 -= da;
					}
					else {
						angle -= da;
						alpha1 += da;
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
				
				
//---------------
				
				float dx = 0.01;
				dx *= (temp.x-msg.pose.position.x) > 0 ? 1 : -1;
				float dy = 0.01;
				dy *= (temp.y-msg.pose.position.y) > 0 ? 1 : -1;
				for (int i = 0; i < 100; i++ ) {
					msg.pose.position.x += dx;
					msg.pose.position.y += dy;

					pubgaz.publish(msg);
					transform.setOrigin( tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
					tf::Quaternion q;
					q.setRPY(0, 0, 0);
					transform.setRotation(q);
					br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "wolf/pose"));

					loop_rate.sleep();

				}
				
//-------------
				while (fabs(angAlf-angle)>0.05)
				{
					float da = 0.01;
					if(alT){
						angle -= da;
						//alpha -= da;
					}
					else {
						angle += da;
						//alpha += da;
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
//--------------
				
			}
			if(fabs(dwl1.x-msg.pose.position.x)<0.5 && fabs(dwl1.y-msg.pose.position.y)<0.5) 
				w1p = true;
			}
			if(!w2p){
			float dx2 =  0.01;
			dx2 *= dwl2.x-msg2.pose.position.x > 0 ? 1 : -1;
			float dy2 =  0.01;
			dy2 *= dwl2.y-msg2.pose.position.y > 0 ? 1 : -1;

			pointS w2;
			w2.x = msg2.pose.position.x+(dx2*100);
			w2.y = msg2.pose.position.y+(dy2*100);
			cout<<"two dist"<<endl;
			if(!dist(w2)){
			for (int i = 0; i < 100; i++ ) {
				msg2.pose.position.x += dx2;
				msg2.pose.position.y += dy2;

				pubgaz2.publish(msg2);
				transform2.setOrigin( tf::Vector3(msg2.pose.position.x, msg2.pose.position.y, 0.0) );
				tf::Quaternion q2;
				q2.setRPY(0, 0, 0);
				transform2.setRotation(q2);
				br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "world", "wolf2/pose"));
				loop_rate.sleep();

			}
			}
			if(fabs(tr2.getOrigin().x()) < 1 && fabs(tr2.getOrigin().y()) < 1) die_sheep++;
			else{
				pointS t;
				t.x = tr2.getOrigin().x() + msg2.pose.position.x;
				t.y = tr2.getOrigin().y() + msg2.pose.position.y;
				pointS temp1;
				pointS temp2;
				pointS temp3;
				pointS temp4;
				pointS arP[4];
				int iT = 0;
				pointS tempF;
				tempF.x= 100;
				tempF.y= 100;

				temp1.x = msg2.pose.position.x+1;
				temp1.y = msg2.pose.position.y;

				temp2.x = msg2.pose.position.x-1;
				temp2.y = msg2.pose.position.y;

				temp3.x = msg2.pose.position.x;
				temp3.y = msg2.pose.position.y+1;

				temp4.x = msg2.pose.position.x;
				temp4.y = msg2.pose.position.y-1;

				if(!dist(temp1)) {
					arP[iT] = temp1;
					iT++;
				}
				if(!dist(temp2)) {
					arP[iT] = temp2;
					iT++;
				}
				if(!dist(temp3)) {
					arP[iT] = temp3;
					iT++;
				}
				if(!dist(temp4)) {
					arP[iT] = temp4;
					iT++;
				}
				while(iT<4)
				{
					arP[iT] = tempF;
					iT++;
				}
				cout<<"two ptr"<<endl;
				pointS temp = distStep(arP, t);

//-----------------

				float xT1 = temp.x-msg2.pose.position.x;
				float yT1 = temp.y-msg2.pose.position.y;		
				//cout<<"x3: "<<x3<<" y3 "<<y3<<endl;
				float aT1 = (xT1);
				float bT1 = (yT1);
				float alpha1 = atan(aT1 / bT1);
				alpha1 = atan(bT1 / aT1);// - alphaT3;
				
				bool alT1 = alpha1 > 0 ? true : false;
				float angAlf = angle2;
				bool alT = alpha1 > 0 ? true : false;
				while ((alpha1 > 0.05 && alT1) || (alpha1 < -0.05 && !alT1))
				{
					float da = 0.01;
					if(alT1){
						angle2 += da;
						alpha1 -= da;
					}
					else {
						angle2 -= da;
						alpha1 += da;
					}
					//cout<<"angle: " <<angle <<"\nalpha: " << alpha <<endl;
					float roll = 0.0; 
					float pitch = 0.0;
					float yaw = angle2;
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

					msg2.pose.orientation.x = sinR * cosH * cosYaw - cosR * sinH * sinYaw;
					msg2.pose.orientation.y = cosR * sinH * cosYaw + sinR * cosH * sinYaw;
					msg2.pose.orientation.z = cosR * cosH * sinYaw - sinR * sinH * cosYaw;
					msg2.pose.orientation.w = cosR * cosH * cosYaw + sinR * sinH * sinYaw;
					pubgaz2.publish(msg2);

					loop_rate.sleep();
				}	
				
				
//---------------
				float dx = 0.01;
				dx *= (temp.x-msg2.pose.position.x) > 0 ? 1 : -1;
				float dy = 0.01;
				dy *= (temp.y-msg2.pose.position.y) > 0 ? 1 : -1;

				for (int i = 0; i < 100; i++ ) {
					msg2.pose.position.x += dx;
					msg2.pose.position.y += dy;

					pubgaz2.publish(msg2);
					transform2.setOrigin( tf::Vector3(msg2.pose.position.x, msg2.pose.position.y, 0.0) );
					tf::Quaternion q2;
					q2.setRPY(0, 0, 0);
					transform2.setRotation(q2);
					br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "world", "wolf2/pose"));
					loop_rate.sleep();


				}
//-------------
				while (fabs(angAlf-angle2)>0.05)
				{
					float da = 0.01;
					if(alT){
						angle2 -= da;
						//alpha -= da;
					}
					else {
						angle2 += da;
						//alpha += da;
					}
				
					//cout<<"angle: " <<angle <<"\nalpha: " << alpha <<endl;
					float roll = 0.0; 
					float pitch = 0.0;
					float yaw = angle2;
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

					msg2.pose.orientation.x = sinR * cosH * cosYaw - cosR * sinH * sinYaw;
					msg2.pose.orientation.y = cosR * sinH * cosYaw + sinR * cosH * sinYaw;
					msg2.pose.orientation.z = cosR * cosH * sinYaw - sinR * sinH * cosYaw;
					msg2.pose.orientation.w = cosR * cosH * cosYaw + sinR * sinH * sinYaw;
					pubgaz2.publish(msg2);

					loop_rate.sleep();
				}
//--------------
				
			}
			
			
			if(fabs(dwl2.x-msg2.pose.position.x)<0.5 && fabs(dwl2.y-msg2.pose.position.y)<0.5) 
				w2p = true;
			}
			if(!w3p){

			float dx3 =  0.01;
			dx3 *= dwl3.x-msg3.pose.position.x > 0 ? 1 : -1;
			float dy3 =  0.01;
			dy3 *= dwl3.y-msg3.pose.position.y > 0 ? 1 : -1;
			
			pointS w3;
			w3.x = msg3.pose.position.x+(dx3*100);
			w3.y = msg3.pose.position.y+(dy3*100);
			cout<<"three dist"<<endl;
			if(!dist(w3)){
			for (int i = 0; i < 100; i++ ) {
				msg3.pose.position.x += dx3;
				msg3.pose.position.y += dy3;

				pubgaz3.publish(msg3);
				transform3.setOrigin( tf::Vector3(msg3.pose.position.x, msg3.pose.position.y, 0.0) );
				tf::Quaternion q3;
				q3.setRPY(0, 0, 0);
				transform3.setRotation(q3);
				br3.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "world", "wolf3/pose"));

				loop_rate.sleep();

			}
			}
			if(fabs(tr3.getOrigin().x()) < 1 && fabs(tr3.getOrigin().y()) < 1) die_sheep++;
			else{
				pointS t;
				t.x = tr3.getOrigin().x() + msg3.pose.position.x;
				t.y = tr3.getOrigin().y() + msg3.pose.position.y;
				pointS temp1;
				pointS temp2;
				pointS temp3;
				pointS temp4;
				pointS arP[4];
				int iT = 0;
				pointS tempF;
				tempF.x= 100;
				tempF.y= 100;
				temp1.x = msg3.pose.position.x+1;
				temp1.y = msg3.pose.position.y;

				temp2.x = msg3.pose.position.x-1;
				temp2.y = msg3.pose.position.y;

				temp3.x = msg3.pose.position.x;
				temp3.y = msg3.pose.position.y+1;

				temp4.x = msg3.pose.position.x;
				temp4.y = msg3.pose.position.y-1;

				if(!dist(temp1)) {
					arP[iT] = temp1;
					iT++;
				}
				if(!dist(temp2)) {
					arP[iT] = temp2;
					iT++;
				}
				if(!dist(temp3)) {
					arP[iT] = temp3;
					iT++;
				}
				if(!dist(temp4)) {
					arP[iT] = temp4;
					iT++;
				}
				while(iT<4)
				{
					arP[iT] = tempF;
					iT++;
				}
				cout<<"three ptr"<<endl;
				pointS temp = distStep(arP, t);
//-----------------

				float xT1 = temp.x-msg3.pose.position.x;
				float yT1 = temp.y-msg3.pose.position.y;		
				//cout<<"x3: "<<x3<<" y3 "<<y3<<endl;
				float aT1 = (xT1);
				float bT1 = (yT1);
				float alpha1 = atan(aT1 / bT1);
				alpha1 = atan(bT1 / aT1);// - alphaT3;
				
				bool alT1 = alpha1 > 0 ? true : false;
				float angAlf = angle3;
				bool alT = alpha1 > 0 ? true : false;
				while ((alpha1 > 0.05 && alT1) || (alpha1 < -0.05 && !alT1))
				{
					float da = 0.01;
					if(alT1){
						angle3 += da;
						alpha1 -= da;
					}
					else {
						angle3 -= da;
						alpha1 += da;
					}
					//cout<<"angle: " <<angle <<"\nalpha: " << alpha <<endl;
					float roll = 0.0; 
					float pitch = 0.0;
					float yaw = angle3;
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

					msg3.pose.orientation.x = sinR * cosH * cosYaw - cosR * sinH * sinYaw;
					msg3.pose.orientation.y = cosR * sinH * cosYaw + sinR * cosH * sinYaw;
					msg3.pose.orientation.z = cosR * cosH * sinYaw - sinR * sinH * cosYaw;
					msg3.pose.orientation.w = cosR * cosH * cosYaw + sinR * sinH * sinYaw;
					pubgaz3.publish(msg3);

					loop_rate.sleep();
				}	
				
				
//---------------

				float dx = 0.01;
				dx *= (temp.x-msg3.pose.position.x) > 0 ? 1 : -1;
				float dy = 0.01;
				dy *= (temp.y-msg3.pose.position.y) > 0 ? 1 : -1;

				for (int i = 0; i < 100; i++ ) {
					msg3.pose.position.x += dx;
					msg3.pose.position.y += dy;

					pubgaz3.publish(msg3);
					transform3.setOrigin( tf::Vector3(msg3.pose.position.x, msg3.pose.position.y, 0.0) );
					tf::Quaternion q3;
					q3.setRPY(0, 0, 0);
					transform3.setRotation(q3);
					br3.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "world", "wolf3/pose"));

					loop_rate.sleep();

				}
//-------------
				while (fabs(angAlf-angle3)>0.05)
				{
					float da = 0.01;
					if(alT){
						angle3 -= da;
						//alpha -= da;
					}
					else {
						angle3 += da;
						//alpha += da;
					}
				
					//cout<<"angle: " <<angle <<"\nalpha: " << alpha <<endl;
					float roll = 0.0; 
					float pitch = 0.0;
					float yaw = angle3;
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

					msg3.pose.orientation.x = sinR * cosH * cosYaw - cosR * sinH * sinYaw;
					msg3.pose.orientation.y = cosR * sinH * cosYaw + sinR * cosH * sinYaw;
					msg3.pose.orientation.z = cosR * cosH * sinYaw - sinR * sinH * cosYaw;
					msg3.pose.orientation.w = cosR * cosH * cosYaw + sinR * sinH * sinYaw;
					pubgaz3.publish(msg3);

					loop_rate.sleep();
				}
//--------------
				
			}
			
			if(fabs(dwl3.x-msg3.pose.position.x)<0.5 && fabs(dwl3.y-msg3.pose.position.y)<0.5) 
				w3p = true;
			}
			pub.publish(msgA);
			
			fl = true;
		loop_rate.sleep();
		}
		
		else
		{
			if(!f){
				fl = false;
				f = true;
			}
			ros::spinOnce();
		}
		ros::spinOnce();
}
}

select_path = true;
	}


	return 0;
}

