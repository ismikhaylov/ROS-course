//
// Created by maksim on 16.11.16.
//

#include "../../../heder/robot/factory/RobotFactory.h"

vector< RobotArcher *> * RobotFactory::
createRobotArcherSquad(NodeHandle nodeHandle,string prefixRobotArcherName,int countRobots,
                       double startLineRobotsX,double startLineRobotsY,
                       double turnAngleRobots,double distanceBetweenRobots,
                       double Z,vector<string>enemiesName) {
    vector< RobotArcher *> * robotArcherSquad=new vector<RobotArcher*>();
    cout<<"turnAngleRobots:"<<turnAngleRobots<<endl;

    double turnAngleBow=turnAngleRobots;
    if(turnAngleRobots<0) {
        turnAngleRobots=fabs(turnAngleRobots);
        turnAngleRobots+= M_PI;
    }
    //cout<<"turnAngleRobots:"<<turnAngleRobots<<endl;
    tf::Quaternion q(tf::Vector3(0, 0, 1), turnAngleRobots);

    geometry_msgs::Quaternion odom;
    tf::quaternionTFToMsg(q, odom);

    geometry_msgs::Pose robotPose;
    robotPose.position.x=startLineRobotsX;
    robotPose.position.y=startLineRobotsY;
   // robotPose.position.z=0.18;//TODO
    robotPose.orientation=odom;

    //cout<<"FR:"<<robotPose.position<<endl<<"End FR";
    if(turnAngleBow>0) {
        turnAngleBow=fabs(turnAngleBow);
        turnAngleBow+= M_PI;
    } else{
        double delta=M_PI_2-fabs(turnAngleBow);
        turnAngleBow= M_PI_2-delta;
    }

    tf::Quaternion q1(tf::Vector3(0, 0, 1), turnAngleBow);

    geometry_msgs::Quaternion odom1;
    tf::quaternionTFToMsg(q1, odom1);


    geometry_msgs::Pose bowPose;
    bowPose.position.x=startLineRobotsX;
    bowPose.position.y=startLineRobotsY;
    bowPose.position.z=0.35;
    bowPose.orientation=odom1;

    int i;
    //RobotArcher **robotArcher=new RobotArcher*[countRobots];
    RobotArcher *robotArcher;
    for(i=0;i<countRobots;i++)
    {
        string buf=prefixRobotArcherName;
        robotArcher=new RobotArcher(nodeHandle,buf.insert(buf.length(),to_string(i+1)),robotPose,bowPose,enemiesName);
        robotPose.position.x+=distanceBetweenRobots;
        bowPose.position.x+=distanceBetweenRobots;
        robotArcherSquad->push_back(robotArcher);
    }
    return robotArcherSquad;
}

RobotCommander* RobotFactory::createRobotCommander(NodeHandle nodeHandle,
                                                   string robotCommanderName,
                                                   double startLineRobotsX,
                                                   double startLineRobotsY,
                                                   double turnAngleRobots,
                                                   vector<string>enemiesName)
{

if(turnAngleRobots<0) {
turnAngleRobots=fabs(turnAngleRobots);
turnAngleRobots+= M_PI;
}
//cout<<"turnAngleRobots:"<<turnAngleRobots<<endl;
tf::Quaternion q(tf::Vector3(0, 0, 1), turnAngleRobots);

geometry_msgs::Quaternion odom;
tf::quaternionTFToMsg(q, odom);

geometry_msgs::Pose robotPose;
robotPose.position.x=startLineRobotsX;
robotPose.position.y=startLineRobotsY;
// robotPose.position.z=0.18;//TODO
robotPose.orientation=odom;
RobotCommander *robotCommander=
        new RobotCommander(nodeHandle,robotCommanderName,robotPose,enemiesName);
    return robotCommander;
}


vector< RobotGuardian *> * RobotFactory::createRobotGuardianSquad(NodeHandle nodeHandle,
                                                               string prefixRobotGuardianName,
                                                               int countRobots,
                                                               double startLineRobotsX, double startLineRobotsY,
                                                               double turnAngleRobots,double distanceBetweenRobots,
                                                               vector<string>enemiesName)
{
    vector< RobotGuardian *> * robotGuardianSquad=new vector<RobotGuardian*>();
    cout<<"turnAngleRobots:"<<turnAngleRobots<<endl;

    double turnAngleSword=turnAngleRobots;
    double shiftY;
    if(turnAngleRobots<0) {
        turnAngleRobots=fabs(turnAngleRobots);
        turnAngleRobots+= M_PI;
        shiftY=-0.45;
    }else  shiftY=0.45;

    tf::Quaternion q(tf::Vector3(0, 0, 1), turnAngleRobots);

    geometry_msgs::Quaternion odom;
    tf::quaternionTFToMsg(q, odom);

    geometry_msgs::Pose robotPose;

    robotPose.position.x=startLineRobotsX;
    robotPose.position.y=startLineRobotsY;
    robotPose.position.z=-0.35;//TODO This Gazebo Bug
    robotPose.orientation=odom;

    if(turnAngleSword<0) {
        turnAngleSword = fabs(turnAngleSword);
        turnAngleSword += M_PI;
    }

    tf::Quaternion q1(tf::Vector3(0, 0,1 ), turnAngleSword);//TODO

    geometry_msgs::Quaternion odom1;
    tf::quaternionTFToMsg(q1, odom1);


    geometry_msgs::Pose swordPose;
    swordPose.position.x=startLineRobotsX;
    swordPose.position.y=startLineRobotsY+shiftY;
    swordPose.position.z=0.35;
    swordPose.orientation=odom1;

    int i;
    RobotGuardian *robotGuardian;
    for(i=0;i<countRobots;i++)
    {
        string buf=prefixRobotGuardianName;
        robotGuardian=new RobotGuardian(nodeHandle,buf.insert(buf.length(),to_string(i+1)),
                                        robotPose,swordPose,enemiesName);
        robotPose.position.x+=distanceBetweenRobots;
        swordPose.position.x+=distanceBetweenRobots;
        robotGuardianSquad->push_back(robotGuardian);
    }
    return robotGuardianSquad;

}

