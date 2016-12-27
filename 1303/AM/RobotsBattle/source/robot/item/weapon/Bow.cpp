//
// Created by maksim on 19.11.16.
//


#include <geometry_msgs/Pose.h>
#include "../../../../heder/robot/item/weapon/Bow.h"
#include "../../../../heder/robot/base/RobotBase.h"
#include "../../../../heder/robot/warior/RobotArcher.h"


Bow::Bow() {}

Bow::Bow(NodeHandle nodeHandle,string pathToModel,string modelName,geometry_msgs::Pose modelPosition,int distanceAttack):
        Weapon(nodeHandle,pathToModel,modelName,modelPosition,distanceAttack)
{

arrow=new Arrow(nodeHandle,string(PATH_TO_USER_DIRECTORY)+string(PATH_TO_ROBOT_ARCHER_PROJECTILE_MODEL),
                string(PREFIX_ROBOT_ARCHER_PROJECTILE)+getModelName(),modelPosition);
}