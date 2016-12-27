//
// Created by maksim on 13.11.16.
//

#ifndef LAB5_ROBOTLOST_H
#define LAB5_ROBOTLOST_H

#include <std_msgs/String.h>
#include "RobotBase.h"

class RobotLost: public RobotBase {
    Publisher publisherHelpMassage;
    Subscriber subscriberDirectionsMassage;
    bool isHelp;
public:
    RobotLost();
    RobotLost(NodeHandle nodeHandle,string robotName,geometry_msgs::Pose currentPosition);
    void performDirections(const std_msgs::String &msgDirections);

    void sendHelpMassage();

    void m1();
};


#endif //LAB5_ROBOTLOST_H
