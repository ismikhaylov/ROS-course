//
// Created by maksim on 17.12.16.
//

#include "../../../../heder/robot/item/weapon/Sword.h"

Sword::Sword() {}
Sword::Sword(NodeHandle nodeHandle, string pathToModel, string modelName, geometry_msgs::Pose modelPosition,int distanceAttack):
Weapon(nodeHandle,pathToModel,modelName,modelPosition,distanceAttack)
{

}