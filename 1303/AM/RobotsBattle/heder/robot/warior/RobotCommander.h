//
// Created by maksim on 16.11.16.
//

#ifndef ROBOTSBATTLE_ROBOTCOMMANDER_H
#define ROBOTSBATTLE_ROBOTCOMMANDER_H
#define PATH_TO_ROBOT_COMMANDER_MODEL "/.gazebo/models/mars_rover/model.sdf"

#include "../base/RobotBase.h"
#include "RobotArcher.h"
#include "RobotGuardian.h"

class RobotCommander: public RobotBase{
    vector< RobotArcher *> *robotArcherSquad;
    vector< RobotGuardian *> *robotGuardianSquad;

    map<string,Publisher*> publisherCommandToArcherMap;
    map<string,Publisher*> publisherCommandToGuardianMap;
    map<string,Publisher*> publisherDeleteEnemyToRobotMap;
public:
    RobotCommander();
    RobotCommander(NodeHandle nodeHandle,string robotName,geometry_msgs::Pose startPosition,
                   vector<string>enemiesName);

public:
  virtual void attack(string &enemyName,double &distanceToEnemy){}

    virtual void live();
    virtual void hideRobotModel(){
        geometry_msgs::Pose &robotPose=getModelPosition();
        robotPose.position.z-=0.35;
        Rate rate(2);
        for(int i=0;i<10;i++) {
            publishModelPosition(getModelName(), robotPose);
            rate.sleep();
            ros::spinOnce();
        }
    }

public:
    void addArcherSquad(vector< RobotArcher *> *_robotArcherSquad);
    void addGuardianSquad(vector< RobotGuardian *> *_robotGuardianSquad);

    vector< RobotArcher *>*getRobotArcherSquad(){
        return robotArcherSquad;
    }
    vector< RobotGuardian *>*getRobotGuardianSquad(){
        return robotGuardianSquad;
    }

  virtual  void deleteEnemy(const std_msgs::String &deleteEnemyName);
public:
    void attackAll();
};


#endif //ROBOTSBATTLE_ROBOTCOMMANDER_H
