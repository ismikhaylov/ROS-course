//
// Created by maksim on 19.11.16.
//

#ifndef ROBOTSBATTLE_ROBOTWEAPON_H
#define ROBOTSBATTLE_ROBOTWEAPON_H


#include "../../base/Service.h"
#include "../Item.h"

class Weapon : public Item{
  int distanceAttack;
public:
    Weapon();
    Weapon(NodeHandle nodeHandle,string pathToModel,string modelName,geometry_msgs::Pose modelPosition,
    int distanceAttack);

public:
    int getDistanceAttack(){
        return distanceAttack;
    }
};


#endif //ROBOTSBATTLE_ROBOTWEAPON_H
