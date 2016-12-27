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

enum class State { Chase, Convoy, Final };

bool isHooked = false;

void statusCallback(const ros_lab5::status::ConstPtr& message)
{
    if (message->is_hooked)
    {
        isHooked = true;
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
    ros::init(argc, argv, "lost_robot");
    ros::NodeHandle node;

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    tf::Transform strayRobotTransform;
    tf::StampedTransform finderRobotTransform;
    double dx, dy, distance;
    double goalX, goalY;
    double hookRange = 0.5;
    State state = State::Chase;
    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_real_distribution<> uniform_dist(-6.0, 6.0);

    // generate robot
    RobotInfo robot("lost_robot");
    robot.setPosition(uniform_dist(engine), uniform_dist(engine));

    goalX = uniform_dist(engine);
    goalY = uniform_dist(engine);

    spawnRobot(node, robot.getName(), "/home/saydos/.gazebo/models/pioneer2dx/yellow_model.sdf", robot.getX(), robot.getY(), 0.0);

    ros::Publisher gazeboPublisher = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    ros::Publisher statusPublisher = node.advertise<ros_lab5::status>("/status", 1000);
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

    strayRobotTransform.setOrigin(tf::Vector3(robot.getX(), robot.getY(), 0.0));
    strayRobotTransform.setRotation(tf::Quaternion(0,0,0,1));
    broadcaster.sendTransform(tf::StampedTransform(strayRobotTransform, ros::Time::now(), "world", "lost_robot"));

    ros::Rate rate(30);
    ROS_INFO("Start");
    while ((state != State::Final) && ros::ok())
    {
        if (state == State::Chase)
        {
            ros::spinOnce();
            // randomMove
            if (isHooked)
            {
                ROS_INFO("Is hooked");
                isHooked = false;
                state = State::Convoy;
            }
            else
            {
                //ROS_INFO("coords: (%f, %f)", robotX, robotY);
                dx = goalX - robot.getX();
                dy = goalY - robot.getY();
                distance = std::sqrt(dx * dx + dy * dy);
                if (distance <= robot.getDistancePrecision())
                {
                    goalX = uniform_dist(engine);
                    goalY = uniform_dist(engine);
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
        }
        else if (state == State::Convoy)
        {
            // listen stray pose
            ros::spinOnce();
            try {
                ros::Time commonTime;
                std::string error;
                listener.waitForTransform("lost_robot", "finder_robot", ros::Time(0), ros::Duration(1.0));
                listener.getLatestCommonTime("lost_robot", "finder_robot", commonTime, &error);
                listener.lookupTransform("lost_robot", "finder_robot", commonTime, finderRobotTransform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                gazeboPublisher.publish(robotState);
                ros::Duration(1.0).sleep();
                continue;
            }
            // move stray robot to finder
            dx = finderRobotTransform.getOrigin().getX();
            dy = finderRobotTransform.getOrigin().getY();
            distance = std::sqrt(dx * dx + dy * dy);
            if (distance <= hookRange) // robot.getDistancePrecision())
            {
                if (isHooked)
                {
                    lab5::status statusMessage;
                    statusMessage.is_hooked = false;
                    statusPublisher.publish(statusMessage);
                    ROS_INFO("Exit");
                    state = State::Final;
                }
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

        strayRobotTransform.setOrigin(tf::Vector3(robot.getX(), robot.getY(), 0.0));
        strayRobotTransform.setRotation(tf::Quaternion(0,0,0,1));
        broadcaster.sendTransform(tf::StampedTransform(strayRobotTransform, ros::Time::now(), "world", "lost_robot"));

        gazeboPublisher.publish(robotState);

        rate.sleep();
    }
    return 0;
}