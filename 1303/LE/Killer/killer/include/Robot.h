#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include "laser_scan/LaserScan.h"

struct Speed {
    double x;
    double y;
    double z;
};

class Robot {
	
protected:
    double sizeX;
    double sizeY;
    double sizeZ;
    std::string model;
    ros::NodeHandle nodeHandler;
    geometry_msgs::Pose position;
    std::string modelName;
    ros::Publisher posePublisher;
    tf::TransformListener listener;
    Speed speed;
    Robot(geometry_msgs::Pose pos, const std::string &model, ros::NodeHandle &handler, const std::string &name);
    Robot(geometry_msgs::Pose pos, const std::string &model, ros::NodeHandle &handler,
             const std::string &name, double sizeX, double sizeY, double sizeZ);
    void addModel();

public:
    void removeModel();
    void changeModel(const std::string &newModel);
	virtual void broadcastPosition();
    void setPosition(const geometry_msgs::Pose &pos);
    geometry_msgs::Pose getPosition();
};

class Killer : public Robot {
    static std::string killerModel;
    ros::Subscriber scanSubscriber;
    ros::Subscriber commandSubscriber;
    ros::Publisher publisher;
    double firingRange;
    float targetDistance;
    bool simpleLaserScan;
    std::map < float, std::vector<std::vector<geometry_msgs::Pose> > > firstObjects;
    std::map < float, std::vector<std::vector<geometry_msgs::Pose> > > secObjects;
    float firstTime;
    float secTime;
    bool stopX;
    bool stopY;
    bool fireCommand;
public:
    Killer(geometry_msgs::Pose pos, ros::NodeHandle &handler, double firingRange);
    void broadcastPosition();
    void followTarget();
    void runX(float posX);
    void runY(float posY);
    bool hasFoundTargetX(float posX);
    bool hasFoundTargetY(float posY);
    void calculateTargetSpeed();
    void analyzeScan(const laser_scan::LaserScan& msg);
    void analyzeScanObjects(const laser_scan::LaserScan& msg, bool dynamic);
    void getCommand(const std_msgs::String &msg);
};


class Barrier : public Robot {
    static std::string barrierModel;
    static unsigned int idCounter;
    
public:
    Barrier(geometry_msgs::Pose pos, ros::NodeHandle &handler, double sizeX, double sizeY,
            double sizeZ);

    double getSizeY();
    double getSizeX();
    double getHeight();
    bool onCoordinate(float y, float z);
};


struct Borders {
    double leftBorder;
    double rightBorder;
    double bottomBorder;
    double upBorder;
};

class Target : public Robot {
    static unsigned int idCounter;
    unsigned int id;
    Borders borders;
    static std::string activeModel;
    static std::string simpleModel;
    bool isActive;


public:
    static double size;
    std::vector<Barrier *> barriers;
    Target(geometry_msgs::Pose pos, ros::NodeHandle &handler, const Borders &borders,
           const Speed &speed);
    void makeStep();
    unsigned int getId() const;
    void activate(bool active);
    void broadcastPosition();
    bool onCoordinate(float y, float z);
};

class Bullet : public Robot {
    static std::string bulletModel;
    ros::Publisher publisher;
public:
    Bullet(geometry_msgs::Pose pos, ros::NodeHandle &handler, const Speed &speed);
    void fly(double x, double y, double z);
};

#endif