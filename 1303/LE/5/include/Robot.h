#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

class Robot {
	std::string model;
protected:
    ros::NodeHandle nodeHandler;
    geometry_msgs::Pose position;
    std::string modelName;
    ros::Publisher posePublisher;
    ros::Subscriber helpSubscriber;
    ros::Publisher publisher;
    tf::TransformListener listener;
    bool saved;

    Robot(geometry_msgs::Pose pos, const std::string &model, const ros::NodeHandle &handler, const std::string &name);
    
public:
	void broadcastPosition();
    void setPosition(const geometry_msgs::Pose &pos);
    geometry_msgs::Pose getPosition();
    void run(const geometry_msgs::Pose &pos, bool freq = true);
    bool isSaved();
};

class SimpleRobot : public Robot {
    static std::string simpleRobotModel;
    
public:
    SimpleRobot(geometry_msgs::Pose pos, const ros::NodeHandle &handler);
    void sendHelpMessage();
    void follow(const std_msgs::String &msg);
 
};

class RescuerRobot : public Robot {
    static std::string rescuerRobotModel;
    geometry_msgs::Pose entryPosition;
    
public:
    RescuerRobot(geometry_msgs::Pose pos, const ros::NodeHandle &handler);
    void findLost(const std_msgs::String &msg);
    bool hasFound(float posX, float posY);
    void returnBack();
};

#endif