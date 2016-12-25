//
// Created by maksim on 19.11.16.
//

#ifndef ROBOTSBATTLE_BOW_H
#define ROBOTSBATTLE_BOW_H
#include <ros/ros.h>

#include "Weapon.h"
#include "../projectile/Arrow.h"

class Bow : public Weapon{
Arrow *arrow;
public:
    Bow();
    Bow(NodeHandle nodeHandle,string pathToModel,string modelName,geometry_msgs::Pose modelPosition,
    int distanceAttack);

public:
    Arrow *getArrow(){
        return arrow;
    }
};


#endif //ROBOTSBATTLE_BOW_H
