//
// Created by maksim on 16.11.16.
//

#include "../../../heder/robot/warior/RobotCommander.h"


RobotCommander::RobotCommander() {}


RobotCommander::RobotCommander(NodeHandle nodeHandle, string robotName, geometry_msgs::Pose startPosition,
                               vector<string>enemiesName)
:RobotBase(nodeHandle,string(PATH_TO_USER_DIRECTORY)+string(PATH_TO_ROBOT_COMMANDER_MODEL),
           robotName,startPosition,enemiesName){
    robotArcherSquad= nullptr;
    robotGuardianSquad= nullptr;
}

void RobotCommander::attackAll() {
    ros::Rate rate(2);
    message::CommandInformation attackCommand;
    attackCommand.commandId=RobotBase::CommandName::CM_ATTACK;
    if(robotArcherSquad!= nullptr)
    for(auto it= robotArcherSquad->begin();it!=robotArcherSquad->end();it++){
        Publisher publisherCommand= *publisherCommandToArcherMap[(*it)->getModelName()];
        publisherCommand.publish(attackCommand);
        rate.sleep();
        ros::spinOnce();
    }
    if(robotGuardianSquad!= nullptr)
    for(auto it= robotGuardianSquad->begin();it!=robotGuardianSquad->end();it++){
        Publisher publisherCommand= *publisherCommandToGuardianMap[(*it)->getModelName()];
        publisherCommand.publish(attackCommand);
        rate.sleep();
        ros::spinOnce();
    }
}

void RobotCommander::deleteEnemy(const std_msgs::String &deleteEnemyName) {
    printf("Commander_Robot %s -> Delete_Enemy:%s\n",getModelName().data(),deleteEnemyName.data.data());
    Rate rate(2);
    for( auto &_pair:publisherDeleteEnemyToRobotMap){
        Publisher *publisherDeleteEnemy=_pair.second;
        publisherDeleteEnemy->publish(deleteEnemyName);
        rate.sleep();
        ros::spinOnce();
    }

}

void RobotCommander::live() {//TODO Добавить броадкастрер
 attackAll();
}

void RobotCommander::addArcherSquad(vector<RobotArcher *> *_robotArcherSquad) {
        robotArcherSquad=_robotArcherSquad;
    Publisher *publisherArcher;
    Publisher *publisherDeleteEnemy;
    NodeHandle &nodeHandle=getNodeHandle();

    for(auto it=robotArcherSquad->begin();it!=robotArcherSquad->end();it++){
        publisherArcher=new Publisher();
        *publisherArcher= nodeHandle.advertise<message::CommandInformation>((*it)->getModelName()+"/command",10);
        publisherCommandToArcherMap.insert(pair<string,Publisher*>((*it)->getModelName(),publisherArcher));

        publisherDeleteEnemy=new Publisher();
        *publisherDeleteEnemy=nodeHandle.advertise<std_msgs::String>((*it)->getModelName()+"/delete_enemy",10);
        publisherDeleteEnemyToRobotMap.insert(pair<string,Publisher*>((*it)->getModelName(),publisherDeleteEnemy));
    }

}

void RobotCommander::addGuardianSquad(vector<RobotGuardian *> *_robotGuardianSquad)
{
    robotGuardianSquad=_robotGuardianSquad;
    Publisher *publisherGuardian;
    Publisher *publisherDeleteEnemy;
    NodeHandle &nodeHandle=getNodeHandle();

    for(auto it=robotGuardianSquad->begin();it!=robotGuardianSquad->end();it++){
        publisherGuardian=new Publisher();
        *publisherGuardian= nodeHandle.advertise<message::CommandInformation>((*it)->getModelName()+"/command",10);
        publisherCommandToGuardianMap.insert(pair<string,Publisher*>((*it)->getModelName(),publisherGuardian));

        publisherDeleteEnemy=new Publisher();
        *publisherDeleteEnemy=nodeHandle.advertise<std_msgs::String>((*it)->getModelName()+"/delete_enemy",10);
        publisherDeleteEnemyToRobotMap.insert(pair<string,Publisher*>((*it)->getModelName(),publisherDeleteEnemy));
    }

}
