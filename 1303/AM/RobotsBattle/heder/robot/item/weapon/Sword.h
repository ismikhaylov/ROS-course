//
// Created by maksim on 17.12.16.
//

#ifndef ROBOTSBATTLE_SWORD_H
#define ROBOTSBATTLE_SWORD_H


#include "Weapon.h"

class Sword: public Weapon {
public:
    Sword();
    Sword(NodeHandle nodeHandle,string pathToModel,string modelName,geometry_msgs::Pose modelPosition,
    int distanceAttack);

};


#endif //ROBOTSBATTLE_SWORD_H
