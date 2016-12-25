//
// Created by maksim on 20.11.16.
//

#ifndef ROBOTSBATTLE_PROJECTILE_H
#define ROBOTSBATTLE_PROJECTILE_H


#include "../Item.h"

class Projectile : public Item{

public:
    Projectile();
    Projectile(NodeHandle nodeHandle,string pathToModel,string modelName,geometry_msgs::Pose modelPosition);
};


#endif //ROBOTSBATTLE_PROJECTILE_H
