//
// Created by maksim on 19.11.16.
//

#include "../../../heder/robot/item/Item.h"



Item::Item() {}

Item::Item(NodeHandle nodeHandle, string pathToModel, string modelName, geometry_msgs::Pose modelPosition)
:Service(nodeHandle,pathToModel,modelName,modelPosition){}