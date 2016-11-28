#include <random>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ModelState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <lab5/status.h>
#include "robot_info.h"

enum class State { Chase, Convoy, Waiting, Final };

bool missionComplete = false;

void statusCallback(const lab5::status::ConstPtr& message)
{
    if (!message->is_hooked)
    {
        missionComplete = true;
    }
}

void spawnRobot(ros::NodeHandle& node, const std::string& name, const std::string& model_path, double x, double y, double z)
{
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot = node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;
    std::ifstream fin(model_path.c_str());
    std::string model_xml;
    std::string buf;
    while(!fin.eof())
    {
        getline(fin, buf);
        model_xml += buf + "\n";
    }
    srv.request.model_xml = model_xml;
    srv.request.model_name = name;
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    srv.request.initial_pose = pose;
    add_robot.call(srv);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "finder_robot");
    ros::NodeHandle node;

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    tf::Transform finderTransform;
    tf::StampedTransform lostRobotTransform;
    double dx, dy, distance;
    double hookRange = 0.5;
    double exitX, exitY;
    State state = State::Chase;
    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_real_distribution<> uniform_dist(-6.0, 6.0);

    RobotInfo robot("finder_robot");
    robot.setPosition(uniform_dist(engine), uniform_dist(engine));

    exitX = robot.getX();
    exitY = robot.getY();

    spawnRobot(node, robot.getName(), "/home/saydos/.gazebo/models/pioneer2dx/green_model.sdf", robot.getX(), robot.getY(), 0.0);
    spawnRobot(node, "exit_point", "/home/saydos/.gazebo/models/cricket_ball/model.sdf", exitX, exitY, 0.0);

    ros::Publisher gazeboPublisher = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    ros::Publisher statusPublisher = node.advertise<lab5::status>("/status", 1000);
    ros::Subscriber statusSupscriber = node.subscribe("/status", 10, &statusCallback);
    sleep(1.0);

    gazebo_msgs::ModelState robotState;
    robotState.model_name = robot.getName();
    robotState.pose.position.x = robot.getX();
    robotState.pose.position.y = robot.getY();
    robotState.pose.orientation.x = 0.0;
    robotState.pose.orientation.y = 0.0;
    robotState.pose.orientation.z = 0.0;
    robotState.pose.orientation.w = 1.0;

    finderTransform.setOrigin(tf::Vector3(robot.getX(), robot.getY(), 0.0));
    finderTransform.setRotation(tf::Quaternion(0,0,0,1));
    broadcaster.sendTransform(tf::StampedTransform(finderTransform, ros::Time::now(), "world", "finder_robot"));

    ros::Rate r(30);
    ROS_INFO("Start");
    while ((state != State::status) && ros::ok())
    {
        if (state == State::Chase)
        {
            // listen lost pose
            try {
                ros::Time commonTime;
                std::string error;
                listener.waitForTransform("finder_robot", "lost_robot", ros::Time(0), ros::Duration(1.0));
                listener.getLatestCommonTime("finder_robot", "lost_robot",commonTime, &error);
                listener.lookupTransform("finder_robot", "lost_robot", commonTime, lostRobotTransform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                gazeboPublisher.publish(robotState);
                ros::Duration(1.0).sleep();
                continue;
            }
            // move finder robot to lost
            dx = lostRobotTransform.getOrigin().getX();// - robotX;
            dy = lostRobotTransform.getOrigin().getY();// - robotY;
            distance = std::sqrt(dx * dx + dy * dy);
            if (distance <= hookRange)
            {
                ROS_INFO("Hooked");
                lab5::status statusMessage;
                statusMessage.is_hooked = true;
                statusPublisher.publish(statusMessage);
                state = State::Convoy;
            }
            robot.updatePosition(dx, dy, distance);
            robotState.pose.position.x = robot.getX();
            robotState.pose.position.y = robot.getY();
            robotState.pose.orientation.z = sin(robot.getCurrentAngle() / 2);
            robotState.pose.orientation.w = cos(robot.getCurrentAngle() / 2);
        }
        else if (state == State::Convoy)
        {
            // move finder robot to exit
            dx = exitX - robot.getX();
            dy = exitY - robot.getY();
            distance = std::sqrt(dx * dx + dy * dy);
            if (distance <= robot.getDistancePrecision())
            {
                ROS_INFO("Waiting...");
                lab5::status statusMessage;
                statusMessage.is_hooked = true;
                statusPublisher.publish(statusMessage);
                sleep(0.5);
                state = State::Waiting;
            }
            else
            {
                robot.updatePosition(dx, dy, distance);
                robotState.pose.position.x = robot.getX();
                robotState.pose.position.y = robot.getY();
                robotState.pose.orientation.z = sin(robot.getCurrentAngle() / 2);
                robotState.pose.orientation.w = cos(robot.getCurrentAngle() / 2);
            }
        }
        else if (state == State::Waiting)
        {
            ros::spinOnce();
            if (missionComplete)
            {
                ROS_INFO("Exit");
                state = State::Final;
            }
        }

        finderTransform.setOrigin(tf::Vector3(robot.getX(), robot.getY(), 0.0));
        finderTransform.setRotation(tf::Quaternion(0,0,0,1));
        broadcaster.sendTransform(tf::StampedTransform(finderTransform, ros::Time::now(), "world", "finder_robot"));

        gazeboPublisher.publish(robotState);
        r.sleep();
    }
    return 0;
}