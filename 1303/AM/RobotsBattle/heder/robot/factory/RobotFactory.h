//
// Created by maksim on 16.11.16.
//

#ifndef ROBOTSBATTLE_ROBOTFACTORY_H
#define ROBOTSBATTLE_ROBOTFACTORY_H

#include "../warior/RobotArcher.h"
#include "../warior/RobotCommander.h"
#include "../warior/RobotGuardian.h"

class RobotFactory{

public:

    static vector< RobotArcher *> * createRobotArcherSquad(NodeHandle nodeHandle,
                                                            string prefixRobotArcherName,
                                                            int countRobots,
                                                            double startLineRobotsX,
                                                            double startLineRobotsY,
                                                            double turnAngleRobots,
                                                            double distanceBetweenRobots,
                                                            double Z,
                                                            vector<string>enemiesName);
    static RobotCommander * createRobotCommander(NodeHandle nodeHandle,
                                                 string robotCommanderName,
                                                 double startLineRobotsX,
                                                 double startLineRobotsY,
                                                 double turnAngleRobots,
                                                 vector<string>enemiesName);

    static vector< RobotGuardian *> * createRobotGuardianSquad(NodeHandle nodeHandle,
                                                             string prefixRobotArcherName,
                                                             int countRobots,
                                                             double startLineRobotsX,
                                                             double startLineRobotsY,
                                                             double turnAngleRobots,
                                                             double distanceBetweenRobots,
                                                             vector<string>enemiesName);
};


#endif //ROBOTSBATTLE_ROBOTFACTORY_H
