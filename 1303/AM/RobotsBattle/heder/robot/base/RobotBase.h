//
// Created by maksim on 13.11.16.
//

#ifndef LAB5_ROBOTBASE_H
#define LAB5_ROBOTBASE_H
#define PATH_TO_USER_DIRECTORY "/home/maksim"
#define TOPIC_2 "topic_2"
#define TOPIC_1 "topic_1"


#include "Service.h"
#include <mutex>
#include <atomic>
#include <thread>
#include "std_msgs/Int8.h"
#include <std_msgs/String.h>
#include "../../message/CommandInformation.h"

class RobotBase :public Service{
    Subscriber subscriberCheckDamage;
    int strength;
public:
    Publisher publisherKill;
    Publisher publisherDeleteItSelf;
    Subscriber deleteEnemySubsciber;
    enum AxisName{AXIS_X,AXIS_Y};
    enum CommandName{CM_NO_COMMAND,CM_ATTACK,CM_MOVE};
    CommandName commandId;
    geometry_msgs::Pose movePoseFromCommand;
    void makeCommand(const message::CommandInformation &commandInformation);
public:
    map<string,Publisher*> publisherEnemiesMap;
    vector<string>enemiesNameVector;
    Subscriber subscriberCommand;
    int range;
public:
    RobotBase();
    RobotBase(NodeHandle nodeHandle,string pathToRobotModel,string robotName,geometry_msgs::Pose startPosition,
              vector<string>enemiesName);

private:
    void turnByAxis(geometry_msgs::Pose poseDirection, AxisName axisName);
    void moveByAxis(geometry_msgs::Pose poseDirection, AxisName axisName);
public:
    void broadcastPosition();
    double getAngle(geometry_msgs::Quaternion quaternionMsg);
    geometry_msgs::Quaternion getQuaternionMessageByZ(double angleRad);
    geometry_msgs::Quaternion getQuaternionMessageByX(double angleRad);
    void turn(double directionAngle);
    void move(geometry_msgs::Pose poseDirection);
    void moveByAngle(tf::TransformListener &listener,tf::StampedTransform &transform,
                    string &enemyName, double &distanceToEnemy, double angle);
    void checkDamage(const geometry_msgs::Pose &attackPose);
    double getDistanceToEnemy(geometry_msgs::Pose &attackPose);
    geometry_msgs::Pose getPoseAttack(tf::StampedTransform &transform);
    tuple<string,geometry_msgs::Pose,double,double> findEnemy(tf::TransformListener &listener,
                                                             tf::StampedTransform &transform);
    tuple<geometry_msgs::Pose,double,double> *getGeometricEnemyInformation(tf::TransformListener &listener,tf::StampedTransform &transform,string &enemyName);
    double getTurnAngleToEnemy(geometry_msgs::Pose &attackPose);

public:
    virtual void hideRobotModel()=0;
    virtual void attack(string &enemyName,double &distanceToEnemy)=0 ;
    virtual void live();
    virtual void changePoseByAxis(double step, AxisName axisName);
    virtual  void deleteEnemy(const std_msgs::String &deleteEnemyName);
    virtual ~RobotBase();
public:
    void setPublisherDeleteItSelf(string topicName){
        publisherDeleteItSelf=getNodeHandle().advertise<std_msgs::String>(topicName.data(),100);
    }
    void setPublishKillTopic(string topicName){
        publisherKill=getNodeHandle().advertise<std_msgs::String>(topicName.data(),100);
    }
    void setStrength(int  _strength=3){
        strength=_strength;
    }
    int getStrength(){
        return strength;
    }
    void decreaseStrength(){
        strength--;
    }
    bool isLive(){
        return strength!=0;
    }
};


#endif //LAB5_ROBOTBASE_H
