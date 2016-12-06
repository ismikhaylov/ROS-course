#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

class Robot {
	
protected:
    std::string model;
    ros::NodeHandle nodeHandler;
    geometry_msgs::Pose position;
    std::string modelName;
    ros::Publisher posePublisher;
    Robot(geometry_msgs::Pose pos, const std::string &model, const ros::NodeHandle &handler, const std::string &name);
    void addModel();

public:
    void removeModel();
    void changeModel(const std::string &newModel);
	void broadcastPosition();
    void setPosition(const geometry_msgs::Pose &pos);
    geometry_msgs::Pose getPosition();
    void run(const geometry_msgs::Pose &pos, bool freq = true);
};

class Killer : public Robot {
    static std::string killerModel;
public:
    Killer(geometry_msgs::Pose pos, const ros::NodeHandle &handler);
};

struct Borders {
    double leftBorder;
    double rightBorder;
    double bottomBorder;
    double upBorder;
};

struct Speed {
    double y;
    double z;
};

class Target : public Robot {
    static unsigned int idCounter;
    unsigned int id;
    Borders borders;
    Speed speed;
    static std::string activeModel;
    static std::string simpleModel;

public:
    static double size;
    Target(geometry_msgs::Pose pos, const ros::NodeHandle &handler, const Borders &borders,
           const Speed &speed);
    void makeStep();
    unsigned int getId() const;
    void activate(bool active);
};

#endif