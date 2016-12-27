//
// Created by maksim on 20.11.16.
//

#include <ros/ros.h>
#include "../../../../heder/robot/item/projectile/Arrow.h"

using  namespace std;
using  namespace ros;
Arrow::Arrow() {}

Arrow::Arrow(NodeHandle nodeHandle,string pathToModel, string arrowName, geometry_msgs::Pose startPosition):
Projectile(nodeHandle,pathToModel,arrowName,startPosition)
{


}
