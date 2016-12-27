//
// Created by maksim on 12.11.16.
//

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <turtlesim/Pose.h>
#include <mutex>
#include "../heder/robot/base/RobotBase.h"
#include "../heder/robot/warior/RobotArcher.h"
#include "../heder/robot/factory/RobotFactory.h"
 #include "thread"
#include "../heder/thread/ThreadPool.h"
//#include "../heder/thread/ThreadPool.h"

#define PREFIX_ROBOT_ARCHER_NAME "t2_archer_"

#define PREFIX_ROBOT_GUARDIAN_NAME "t2_guardian_"

#define PREFIX_ROBOT_ENEMIES_NAME "t1_"

#define ROBOT_COMMANDER_NAME "c2"
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc,argv,"node2");
    ros::NodeHandle robotNode;

    vector<RobotArcher*> *robotArcherSquad;
    vector<RobotGuardian*> *robotGuardianSquad;

    RobotCommander *robotCommander;

    ThreadPool *threadPoolForAllRobots;

    int countArcher=1;
    int countGuardian=0;

    int countArcherEnemy=1;
    int countGuardianEnemy=0;

    string topicKillName="kill_2";
    string enemyCommanderName="c1";
    vector<string>enemiesNameVector;

    geometry_msgs::Pose poseDirection;
    poseDirection.position.x = -5;
    poseDirection.position.y = 5;

    for(int i=0;i<countArcherEnemy;i++)
        enemiesNameVector.push_back(string(PREFIX_ROBOT_ENEMIES_NAME)+string("archer_")+string(to_string(i+1)));

    for(int i=0;i<countGuardianEnemy;i++)
        enemiesNameVector.push_back(string(PREFIX_ROBOT_ENEMIES_NAME)+string("guardian_")+string(to_string(i+1)));

    //enemiesNameVector.push_back("c1");

    robotArcherSquad=RobotFactory::createRobotArcherSquad(robotNode,PREFIX_ROBOT_ARCHER_NAME,
                                                          countArcher,0,7,-1.57,2,2,
                                                          enemiesNameVector);

    robotGuardianSquad=RobotFactory::createRobotGuardianSquad(robotNode,PREFIX_ROBOT_GUARDIAN_NAME,
                                                              countGuardian,0,5,-1.57,2,
                                                          enemiesNameVector);



    robotCommander=RobotFactory::createRobotCommander(robotNode,ROBOT_COMMANDER_NAME,2,9,M_PI,
                                                      enemiesNameVector);
    robotCommander->addArcherSquad(robotArcherSquad);
    robotCommander->addGuardianSquad(robotGuardianSquad);

    tf::TransformListener listener;
    tf::StampedTransform transform;
    while (ros::ok()) {
        try {
            listener.lookupTransform("world", "t1_archer_1", ros::Time(0), transform);
            break;
        } catch (tf::TransformException &ex) {
            continue;
        }
    }
    threadPoolForAllRobots=new ThreadPool(robotNode,topicKillName, robotCommander,enemyCommanderName);

/*/
    ros::Rate rate(2);

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

