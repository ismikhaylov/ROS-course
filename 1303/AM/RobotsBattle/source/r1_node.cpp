//
// Created by maksim on 12.11.16.
//



#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#include <std_msgs/String.h>

#include "../heder/robot/base/RobotBase.h"
#include "../heder/robot/factory/RobotFactory.h"
#include "../heder/thread/ThreadPool.h"

using namespace std;

#define ROBOT_COMMANDER_NAME "c1"
#define PREFIX_ROBOT_ARCHER_NAME "t1_archer_"
#define PREFIX_ROBOT_GUARDIAN_NAME "t1_guardian_"

#define PREFIX_ROBOT_ENEMIES_NAME "t2_"


int main(int argc, char **argv) {
    ros::init(argc,argv,"node1");
    ros::NodeHandle robotNode;
    vector<RobotGuardian*> *robotGuardianSquad;
    vector<RobotArcher*> *robotArcherSquad;
    RobotCommander *robotCommander;

    ros::Rate rate(2);

    int countArcher=1;
    int countGuardian=0;

    int countArcherEnemy=1;
    int countGuardianEnemy=0;

    string topicKillName="kill_1";
    string enemyCommanderName="c2";
    vector<string>enemiesNameVector;

    for(int i=0;i<countArcherEnemy;i++)
        enemiesNameVector.push_back(string(PREFIX_ROBOT_ENEMIES_NAME)+string("archer_")+string(to_string(i+1)));

    for(int i=0;i<countGuardianEnemy;i++)
        enemiesNameVector.push_back(string(PREFIX_ROBOT_ENEMIES_NAME)+string("guardian_")+string(to_string(i+1)));


    //enemiesNameVector.push_back("c2");

    robotArcherSquad=RobotFactory::createRobotArcherSquad(
            robotNode,PREFIX_ROBOT_ARCHER_NAME,countArcher,0,-2,1.57,2,0.3,enemiesNameVector);

    robotCommander=RobotFactory::createRobotCommander(robotNode,ROBOT_COMMANDER_NAME,2,-5,0,
                                                      enemiesNameVector);

    robotGuardianSquad=RobotFactory::createRobotGuardianSquad(robotNode,PREFIX_ROBOT_GUARDIAN_NAME,
                                                              countGuardian,0,0,1.57,2,
                                                              enemiesNameVector);

    robotCommander->addArcherSquad(robotArcherSquad);
    robotCommander->addGuardianSquad(robotGuardianSquad);

    geometry_msgs::Pose poseDirection;
    poseDirection.position.x=-5;
    poseDirection.position.y=5;
    ThreadPool threadPoolForAllRobots(robotNode,topicKillName,robotCommander,enemyCommanderName);
    ///Attack Command
/*/
    message::CommandInformation attackCommand;
    attackCommand.commandId=RobotBase::CommandName::CM_ATTACK;
    Publisher publisherCommand=robotNode.advertise<message::CommandInformation>((*robotArcherSquad)[0]->getModelName()+"/command",10);
    publisherCommand.publish(attackCommand);
/*/

/*/
    message::CommandInformation attackCommand1;
    attackCommand1.commandId=RobotBase::CommandName::CM_MOVE;
    attackCommand1.actionPoseX=poseDirection.position.x;
    attackCommand1.actionPoseY=poseDirection.position.y;
    Publisher publisherCommand1=robotNode.advertise<message::CommandInformation>((robotCommander)->getModelName()+"/command",10);
    publisherCommand1.publish(attackCommand1);
/*/
    ///Move Command
/*/
    message::CommandInformation moveCommand;
    moveCommand.commandId=RobotBase::CommandName::CM_MOVE;
    moveCommand.actionPoseX=poseDirection.position.x;
    moveCommand.actionPoseY=poseDirection.position.y;
    Publisher publisherCommand=robotNode.advertise<message::CommandInformation>((*robotGuardianSquad)[0]->getModelName()+"/command",10);
    publisherCommand.publish(moveCommand);

    rate.sleep();
/*/
    ros::spin();
    return 0;
}
