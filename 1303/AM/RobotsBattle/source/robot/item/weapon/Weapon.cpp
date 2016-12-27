//
// Created by maksim on 19.11.16.
//

#include "../../../../heder/robot/item/weapon/Weapon.h"



Weapon::Weapon(){}

Weapon::Weapon(NodeHandle nodeHandle,string pathToModel,string modelName,geometry_msgs::Pose modelPosition,
int _distanceAttack):distanceAttack(_distanceAttack),
        Item(nodeHandle,pathToModel,modelName,modelPosition){}