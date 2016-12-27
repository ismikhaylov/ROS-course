//
// Created by maksim on 06.12.16.
//

#ifndef ROBOTSBATTLE_THREADPOOL_H
#define ROBOTSBATTLE_THREADPOOL_H

#include <iostream>
#include <vector>
#include <boost/smart_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/ref.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "../robot/warior/RobotCommander.h"

using std::vector;
using std::pair;
using boost::thread;
class ThreadPool {
    map<string,pair<boost::thread,RobotBase*> >threadMap;
    Publisher publisherToEnemyCommander;
    string enemyCommanderName;
    Subscriber subscribeKill;
    Subscriber subscribeDelete;
public:
    ThreadPool(NodeHandle nodeHandle,string topicKillName, RobotCommander *robotCommander,string _enemyCommanderName);
    void deleteRobotByName(std_msgs::String robotName) ;
    void killRobotByName(std_msgs::String robotName) ;
};


#endif //ROBOTSBATTLE_THREADPOOL_H
