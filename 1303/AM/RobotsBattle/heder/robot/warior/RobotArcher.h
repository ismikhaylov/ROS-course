//
// Created by maksim on 16.11.16.
//

#ifndef ROBOTSBATTLE_ROBOTARCHER_H
#define ROBOTSBATTLE_ROBOTARCHER_H

#define PATH_TO_ROBOT_ARCHER_MODEL "/.gazebo/models/pioneer3at/model.sdf"


#define PATH_TO_ROBOT_ARCHER_BOW_MODEL "/.gazebo/models/t_brace_part/model.sdf"


#define PREFIX_ROBOT_ARCHER_BOW "bow_"

#include <gazebo_msgs/DeleteModel.h>
#include <std_msgs/String.h>
#include "../base/RobotBase.h"
#include "../item/weapon/Bow.h"

class RobotArcher :public RobotBase{
private:
    const double poseRobotByZ=0.18;
    const double poseBowByZ=0.35;
    Bow *bow;
public:
    RobotArcher();
    RobotArcher(NodeHandle nodeHandle,string robotName,geometry_msgs::Pose startPosition,
               geometry_msgs::Pose bowPosition,vector<string>enemiesName);
public:
    virtual  void attack(string &enemyName,double &distanceToEnemy);
    virtual geometry_msgs::Pose getAttackPosition(double angleRad,int step);
    virtual  void setModelPosition(geometry_msgs::Pose &pose);
    virtual  void publishModelPosition(string modelName,geometry_msgs::Pose modelPosition);
    virtual  void changePoseByAxis(double step, AxisName axisName);
    virtual void hideRobotModel(){
        geometry_msgs::Pose &pose=getModelPosition();
        pose.position.z-=0.35;
        Rate rate(30);
        for(int i=0;i<10;i++)
        {
            RobotBase::setModelPosition(pose);
            bow->setModelPosition(pose);
            bow->getArrow()->setModelPosition(pose);
            publishModelPosition(getModelName(),pose);
            rate.sleep();
            ros::spinOnce();
        }
    }

public:
    virtual   ~RobotArcher();
};


#endif //ROBOTSBATTLE_ROBOTARCHER_H
