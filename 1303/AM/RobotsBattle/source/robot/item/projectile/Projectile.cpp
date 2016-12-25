//
// Created by maksim on 20.11.16.
//

#include "../../../../heder/robot/item/projectile/Projectile.h"



Projectile::Projectile() {}

Projectile::Projectile(NodeHandle nodeHandle, string pathToModel, string modelName,
                       geometry_msgs::Pose modelPosition)
:Item(nodeHandle,pathToModel,modelName,modelPosition){}