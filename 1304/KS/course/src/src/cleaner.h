#include <ros/ros.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/ModelState.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <string.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>

#include <gazebo/msgs/laserscan_stamped.pb.h>

using namespace std;

#define PATH_MODEL "/home/sergei/workspace/src/5/model/create/model.sdf"
#define G_TO_RAD 0.0174533
#define DMOVE 0.1
#define DROTATE 5


ros::Publisher pub;
gazebo_msgs::ModelState msg;
gazebo::transport::SubscriberPtr subLaserScan;
ros::ServiceClient gazeboDeleteModel;



float botAngle = 0;
bool isFrontWall = false;

void updateWallInfo(ConstLaserScanStampedPtr& _msg)
{
    gazebo::msgs::LaserScan scan = _msg->scan();
    int count = scan.count();

    for (int i = 0; i < count; ++i) {
        if (scan.ranges(i) < 1.0) {
            isFrontWall = true;
            std::cout << "WALL!" << std::endl;
            return ;
        }
    }
    isFrontWall = false;
    std::cout << "NOT WALL!!" << std::endl;
}


void spawnModel(const char* name)
{
    ros::NodeHandle nh;
    ros::Time::init();

    ROS_INFO( "WaitForService: [gazebo/spawn_sdf_model]" );
    ros::service::waitForService( "gazebo/spawn_sdf_model" );
    ROS_INFO( "WaitForService: [gazebo/delete_model]" );
    ros::service::waitForService("gazebo/delete_model");

    ros::ServiceClient gazeboSpawnModel = 
             nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
              
    ifstream fin( PATH_MODEL );
 
    string model;
    string buf;
    while(!fin.eof()){
        getline(fin, buf);
        model += buf + "\n";
    }

    gazebo_msgs::SpawnModel srv;
    srv.request.model_xml = model;
    srv.request.model_name = name;
    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.orientation.z = 0;
    msg.pose = pose;
    msg.model_name = name;
    srv.request.initial_pose = pose;
    gazeboSpawnModel.call(srv);

    pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    ROS_INFO( "SpawnModel ready" );
    gazeboDeleteModel = 
             nh.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");

    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    subLaserScan =
                node->Subscribe("~/cleaner/base/laser/scan", updateWallInfo);
    ROS_INFO( "Subscribe to laser scan" );
}

void removeModel(const char* name)
{
    gazebo_msgs::DeleteModel srv;
    srv.request.model_name = name;
    gazeboDeleteModel.call(srv);
}

void moveTo(float distance)
{
    float x = distance * cos( botAngle * G_TO_RAD ) + msg.pose.position.x;
    float y = distance * sin( botAngle * G_TO_RAD ) + msg.pose.position.y;

    msg.pose.position.x = x;
    msg.pose.position.y = y;
    pub.publish( msg );
}

void rotate(float angle)
{   
    botAngle += angle;
    if (botAngle < 0) {
        botAngle += 360;
    } else if (botAngle > 360) {
        botAngle -= 360;
    }

    tf::Quaternion q(tf::Vector3(0, 0, 1), botAngle * G_TO_RAD);
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(q, odom_quat);
    msg.pose.orientation = odom_quat;
    pub.publish( msg );
}


void init(const char* name)
{
	spawnModel(name);
	rotate( 0 );
}


void executeAlg(float& dis, float& angle)
{
    static float s = 0;
    static int flag = 1;
    dis = angle = 0.0;

    if (!isFrontWall) {
        dis = 0.5;
        s += dis;
    } else if (s > 0) {
        if (s < 5) {
            if (flag == 1) {
                angle = 90;
                flag = 0;
            } else {
                angle = -90;
                flag = 1;
            }
        } else {
            float tmpAngle = 180 - ((float)15/s) * 180 /3.14;
            if (flag == 1) {
                angle = tmpAngle;
                flag == 0;
            } else {
                angle = -tmpAngle;
                flag = 1;
            }
        }
        s = 0;
    } else {
        angle = 90;
    }
}

void execute()
{
    static int state = 0;
    static float taskDist = 0;
    static float taskAngle = 0;

    std::cout << "Step ";
    if (taskDist > 0 && !isFrontWall) {
        std::cout << "move front: " << taskDist << std::endl;
        moveTo( DMOVE );
        taskDist -= DMOVE;
    } else if (taskAngle > 0) {        
        std::cout << "rotate: " << taskAngle << std::endl;
        rotate(DROTATE);
        taskAngle -= DROTATE;
        if (taskAngle < 0) {
            taskAngle = 0;
        }
    } else if (taskAngle < 0) {
        std::cout << "rotate: " << taskAngle << std::endl;
        rotate(-DROTATE);
        taskAngle += DROTATE;
        if (taskAngle > 0) {
            taskAngle = 0;
        }
    } else {
        executeAlg( taskDist, taskAngle );
        std::cout << "executeAlg: " << taskDist << " " << taskAngle << std::endl;
    }
}
