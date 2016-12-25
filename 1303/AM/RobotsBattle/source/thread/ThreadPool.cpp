//
// Created by maksim on 06.12.16.
//

#include "../../heder/thread/ThreadPool.h"


ThreadPool::ThreadPool(NodeHandle nodeHandle,string topicKillName, RobotCommander *robotCommander,string _enemyCommanderName){
//enemyCommanderName=_enemyCommanderName;
subscribeKill=nodeHandle.subscribe(topicKillName,10,&ThreadPool::killRobotByName,this);
subscribeDelete=nodeHandle.subscribe(topicKillName+"/delete",10,&ThreadPool::deleteRobotByName,this);

publisherToEnemyCommander=nodeHandle.advertise<std_msgs::String>(_enemyCommanderName+"/delete_enemy",10);

vector<RobotArcher*>* robotArcherSquad=robotCommander->getRobotArcherSquad();
vector<RobotGuardian*>* robotGuardianSquad=robotCommander->getRobotGuardianSquad();
//thread threadForCommander= thread(&RobotBase::live,robotCommander);

threadMap.insert(pair<string,pair<boost::thread,RobotBase*> >
        (robotCommander->getModelName(),
         pair<boost::thread,RobotBase*>
                 (thread(&RobotBase::live,robotCommander),robotCommander)));
///Назначаем топик topicKillName для всех роботов применяется @RobotBase::checkDamage
if(robotArcherSquad!=NULL) {
cout<<"Archer_Squad:"<<robotArcherSquad->size()<<" "<<robotArcherSquad<<endl;
for (auto &robot:*robotArcherSquad) {
robot->setPublishKillTopic(topicKillName);
robot->setPublisherDeleteItSelf(topicKillName + "/delete");

threadMap.insert(pair<string, pair<boost::thread, RobotBase *> >
        (robot->getModelName(),
         pair<boost::thread, RobotBase *>
                             (thread(&RobotBase::live, robot), robot)));
}
}
if(robotGuardianSquad!= nullptr) {
for (auto &robot:*robotGuardianSquad) {
robot->setPublishKillTopic(topicKillName);
robot->setPublisherDeleteItSelf(topicKillName + "/delete");

threadMap.insert(pair<string, pair<boost::thread, RobotBase *> >
        (robot->getModelName(),
         pair<boost::thread, RobotBase *>
                             (thread(&RobotBase::live, robot), robot)));
}
}

}

void ThreadPool::deleteRobotByName(std_msgs::String robotName) {
    cout << "NAME FOR DELETE:" << robotName << endl;
    pair<boost::thread, RobotBase *> &_pair = threadMap[robotName.data];
    RobotBase*robot=_pair.second;
    robot->hideRobotModel();
}

void ThreadPool::killRobotByName(std_msgs::String robotName) {
    cout << "NAME FOR KILL:" << robotName << endl;
    Rate rate(2);
    publisherToEnemyCommander.publish(robotName);
    rate.sleep();
    ros::spinOnce();
    pair<boost::thread,RobotBase*> &_pair=threadMap[robotName.data];
    boost::thread &_thread=_pair.first;
    _thread.interrupt();
}