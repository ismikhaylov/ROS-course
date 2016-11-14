#include <random>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ModelState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros_lab5/succeed.h>
#include "robot_info.h"

enum class State { Chase, Convoy, Waiting, Succeed };

bool missionComplete = false;

void succeedCallback(const ros_lab5::succeed::ConstPtr& message)
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
    ros::init(argc, argv, "robot_rescuer");
    ros::NodeHandle node;

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    tf::Transform rescuerTransform;
    tf::StampedTransform strayRobotTransform;
    double dx, dy, distance;
    double hookRange = 0.5;
    double exitX, exitY;
    State state = State::Chase;
    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_real_distribution<> uniform_dist(-6.0, 6.0);

    RobotInfo robot("robot_rescuer");
    robot.setPosition(uniform_dist(engine), uniform_dist(engine));

    exitX = robot.getX();
    exitY = robot.getY();

    spawnRobot(node, robot.getName(), "/home/saydos/.gazebo/models/pioneer2dx/green_model.sdf", robot.getX(), robot.getY(), 0.0);
    spawnRobot(node, "exit_point", "/home/saydos/.gazebo/models/cricket_ball/model.sdf", exitX, exitY, 0.0);

    ros::Publisher gazeboPublisher = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    ros::Publisher succeedPublisher = node.advertise<ros_lab5::succeed>("/succeed", 1000);
    ros::Subscriber succeedSupscriber = node.subscribe("/succeed", 10, &succeedCallback);
    sleep(1.0);

    gazebo_msgs::ModelState robotState;
    robotState.model_name = robot.getName();
    robotState.pose.position.x = robot.getX();
    robotState.pose.position.y = robot.getY();
    robotState.pose.orientation.x = 0.0;
    robotState.pose.orientation.y = 0.0;
    robotState.pose.orientation.z = 0.0;
    robotState.pose.orientation.w = 1.0;

    rescuerTransform.setOrigin(tf::Vector3(robot.getX(), robot.getY(), 0.0));
    rescuerTransform.setRotation(tf::Quaternion(0,0,0,1));
    broadcaster.sendTransform(tf::StampedTransform(rescuerTransform, ros::Time::now(), "world", "robot_rescuer"));

    ros::Rate r(30);
    ROS_INFO("Start");
    while ((state != State::Succeed) && ros::ok())
    {
        if (state == State::Chase)
        {
            // listen stray pose
            try {
                ros::Time commonTime;
                std::string error;
                listener.waitForTransform("robot_rescuer", "stray_robot", ros::Time(0), ros::Duration(1.0));
                listener.getLatestCommonTime("robot_rescuer", "stray_robot",commonTime, &error);
                listener.lookupTransform("robot_rescuer", "stray_robot", commonTime, strayRobotTransform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                gazeboPublisher.publish(robotState);
                ros::Duration(1.0).sleep();
                continue;
            }
            // move rescuer robot to stray
            dx = strayRobotTransform.getOrigin().getX();// - robotX;
            dy = strayRobotTransform.getOrigin().getY();// - robotY;
            distance = std::sqrt(dx * dx + dy * dy);
            if (distance <= hookRange)
            {
                ROS_INFO("Hooked");
                ros_lab5::succeed succeedMessage;
                succeedMessage.is_hooked = true;
                succeedPublisher.publish(succeedMessage);
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
            // move rescuer robot to exit
            dx = exitX - robot.getX();
            dy = exitY - robot.getY();
            distance = std::sqrt(dx * dx + dy * dy);
            if (distance <= robot.getDistancePrecision())
            {
                ROS_INFO("Waiting...");
                ros_lab5::succeed succeedMessage;
                succeedMessage.is_hooked = true;
                succeedPublisher.publish(succeedMessage);
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
                state = State::Succeed;
            }
        }

        rescuerTransform.setOrigin(tf::Vector3(robot.getX(), robot.getY(), 0.0));
        rescuerTransform.setRotation(tf::Quaternion(0,0,0,1));
        broadcaster.sendTransform(tf::StampedTransform(rescuerTransform, ros::Time::now(), "world", "robot_rescuer"));

        gazeboPublisher.publish(robotState);
        r.sleep();
    }
    return 0;
}
