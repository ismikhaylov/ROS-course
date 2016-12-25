//
// Created by maksim on 19.11.16.
//

#ifndef ROBOTSBATTLE_ROBOTITEM_H
#define ROBOTSBATTLE_ROBOTITEM_H


#include "../base/Service.h"

class Item : public Service {

public:
    Item();
    Item(NodeHandle nodeHandle,string pathToModel,string modelName,geometry_msgs::Pose modelPosition);

};


#endif //ROBOTSBATTLE_ROBOTITEM_H
