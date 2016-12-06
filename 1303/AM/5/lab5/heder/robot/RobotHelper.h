//
// Created by maksim on 13.11.16.
//

#ifndef LAB5_ROBOTHELPER_H
#define LAB5_ROBOTHELPER_H

#include "RobotBase.h"
#include "std_msgs/String.h"
class RobotHelper:public RobotBase {
    Subscriber subscriberHelper;
    Publisher publisherHelper;
    std_msgs::String robotLostName;
public:
RobotHelper();
  RobotHelper(NodeHandle nodeHandle,string robotName,geometry_msgs::Pose startPosition);
    void findRobotLost(const std_msgs::String &msgHelp);
    void goBack();
};


#endif //LAB5_ROBOTHELPER_H
