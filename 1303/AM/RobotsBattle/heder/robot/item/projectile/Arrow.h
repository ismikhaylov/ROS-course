//
// Created by maksim on 20.11.16.
//

#ifndef ROBOTSBATTLE_ARROW_H
#define ROBOTSBATTLE_ARROW_H

#include <geometry_msgs/Pose.h>

#define PATH_TO_ROBOT_ARCHER_PROJECTILE_MODEL "/.gazebo/models/cricket_ball/model.sdf"
#define PREFIX_ROBOT_ARCHER_PROJECTILE "projectile_"

#include "../weapon/Weapon.h"
#include "Projectile.h"

class Arrow :public Projectile{
public:
    Arrow();
    Arrow(NodeHandle nodeHandle,string pathToModel,string arrowName,geometry_msgs::Pose startPosition);

};


#endif //ROBOTSBATTLE_ARROW_H
