//
// Created by maksim on 13.11.16.
//

#include <gazebo_msgs/DeleteModel.h>
#include <std_msgs/String.h>
 #include "../../../heder/robot/base/RobotBase.h"

RobotBase::RobotBase(){
}
RobotBase::RobotBase(NodeHandle nodeHandle,string pathToRobotModel,string robotName,geometry_msgs::Pose currentPosition,
                     vector<string>enemiesName):
Service(nodeHandle,pathToRobotModel,robotName,currentPosition){
    commandId=CM_NO_COMMAND;
    this->strength=1;
    enemiesNameVector=enemiesName;
    cout<<"RobotName:"<<getModelName()<<endl;
    cout<<"EnemiesName:"<<endl;
    Publisher *publisher;
    for(auto it=enemiesNameVector.begin();it!=enemiesNameVector.end();it++) {
        cout << "--->Enemies:" << (*it) << endl;
        publisher=new Publisher();
         *publisher=nodeHandle.advertise<geometry_msgs::Pose>(*it+"/check_damage",100);
        publisherEnemiesMap.insert(pair<string,Publisher*>(*it,publisher));
    }
    deleteEnemySubsciber=nodeHandle.subscribe(robotName+"/delete_enemy",10,&RobotBase::deleteEnemy,this);
    subscriberCheckDamage=nodeHandle.subscribe(getModelName()+"/check_damage",10,&RobotBase::checkDamage,this);
    subscriberCommand=getNodeHandle().subscribe(getModelName()+"/command",10,&RobotBase::makeCommand,this);
}
void RobotBase::deleteEnemy(const std_msgs::String &deleteEnemyName) {
string enemyName=deleteEnemyName.data;
    int index=0;
    bool isFind= false;
    printf("Base_Robot %s -> Delete_Enemy:%s\n",getModelName().data(),deleteEnemyName.data.data());
    for(auto it=enemiesNameVector.begin();it!=enemiesNameVector.end();it++){
        if((*it)==enemyName){
            printf("FIND DELETE NAME\n");
            isFind=true;
            break;
        }
        index++;
    }
    if(isFind)  enemiesNameVector.erase(enemiesNameVector.begin()+index);

}

void RobotBase::broadcastPosition(){
    geometry_msgs::Pose &robotPosition=getModelPosition();
    //sleep(1);//TODO DON'T REMOVE
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(robotPosition.position.x, robotPosition.position.y, robotPosition.position.z) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", getModelName()));
}

geometry_msgs::Pose RobotBase::getPoseAttack(tf::StampedTransform &transform) {
    geometry_msgs::Pose attackPose;
    double transformX=transform.getOrigin().getX();
    double transformY=transform.getOrigin().getY();
    double robotPoseX=getModelPosition().position.x;
    double robotPoseY=getModelPosition().position.y;
    attackPose.position.x=robotPoseX+transformX;
    attackPose.position.y=robotPoseY+transformY;
    return attackPose;
}

double RobotBase::getTurnAngleToEnemy(geometry_msgs::Pose &attackPose) {
    double angleRad = getAngle(getModelPosition().orientation);

    double robotPoseX=getModelPosition().position.x;
    double robotPoseY=getModelPosition().position.y;
    double robotLookPoseX = robotPoseX + 5 * cos(angleRad);
    double robotLookPoseY = robotPoseY + 5 * sin(angleRad);
    double attackPoseX=attackPose.position.x;
    double attackPoseY=attackPose.position.y;

    double A1=robotPoseY-robotLookPoseY;
    double B1=robotLookPoseX-robotPoseX;
    double C1=-(A1*robotPoseX+B1*robotPoseY);

    double A2=robotPoseY-attackPoseY;
    double B2=attackPoseX-robotPoseX;

    double valCos=(A1*A2+B1*B2)/(sqrt(pow(A1,2)+pow(B1,2))*sqrt(pow(A2,2)+pow(B2,2)));
    double turnAngle=acos(valCos);

    if ((A1 * attackPoseX + B1 * attackPoseY + C1) < 0)turnAngle *= -1;

    return turnAngle;
}

tuple<geometry_msgs::Pose,double,double> *RobotBase::getGeometricEnemyInformation(
        tf::TransformListener &listener,
        tf::StampedTransform &transform,
        string &enemyName) {
    geometry_msgs::Pose attackPose;
    double distanceToEnemy;
    double turnAngleEnemy;

    try {
        broadcastPosition();
        listener.lookupTransform(getModelName(),enemyName, ros::Time(0), transform);
    }catch (tf::TransformException &ex) {
        broadcastPosition();
        string strErr=ex.what();
        ROS_INFO("%d:(%s)-->(%s)ERROR:%s\n",enemiesNameVector.size(),getModelName().data(),enemyName.data(),ex.what());
        ros::Duration(1.0).sleep();
        return NULL;
    }
    attackPose = getPoseAttack(transform);
    distanceToEnemy=getDistanceToEnemy(attackPose);
    turnAngleEnemy = getTurnAngleToEnemy(attackPose);
    return new tuple<geometry_msgs::Pose,double,double>(attackPose,distanceToEnemy,turnAngleEnemy);
}

void RobotBase::moveByAngle(tf::TransformListener &listener,tf::StampedTransform &transform,
                            string &enemyName,double &distanceToEnemy, double angleRad) {//TODO add listener and transform
    geometry_msgs::Pose robotPosition=getModelPosition();
    geometry_msgs::Pose attackPose;
    double startX=robotPosition.position.x;
    double startY=robotPosition.position.y;
    double turnAngleEnemy;
    Rate rateMove(300);
    double delta=0.001;

    tuple<geometry_msgs::Pose,double,double> *geometricEnemyInformation;
    geometry_msgs::Pose &robotPose=getModelPosition();
    int count=0;
    for (double j = 0;/*/ j<distanceToEnemy/*//*/distanceToEnemy>range/*/;j += delta) {
        geometricEnemyInformation=  getGeometricEnemyInformation(listener,transform,enemyName);
        if(geometricEnemyInformation==NULL) {
            if(enemiesNameVector.size()==0)return;
            count++;
            cout<<"Size:"<<enemiesNameVector.size()<<endl;
            continue;
        }
if(count==10)break;
        if(enemiesNameVector.size()==0) return;

        attackPose= get<0>(*geometricEnemyInformation);
        distanceToEnemy= get<1>(*geometricEnemyInformation);
        if(distanceToEnemy<=range)break;
        turnAngleEnemy= get<2>(*geometricEnemyInformation);

        robotPosition.position.x =startX + j * cos(angleRad);
        robotPosition.position.y =startY + j * sin(angleRad);
       // cout<<"Delta:"<<robotPosition.position.x<<" "<<robotPosition.position.y <<endl;
        setModelPosition(robotPosition);
        publishModelPosition(getModelName(),robotPosition);

        rateMove.sleep();
        ros::spinOnce();
        if(turnAngleEnemy>0.1)turn(angleRad+turnAngleEnemy);
        boost::this_thread::interruption_point();
    }
    printf("Attacker:%s\n",getModelName().data());
    printf("Enemy Name:%s\n",enemyName.data());
    printf("Robot Pose:X=%f Y=%f\n",robotPose.position.x,robotPose.position.y);
    printf("Enemy Pose:X=%f Y=%f\n",attackPose.position.x,attackPose.position.y);
    printf("Enemy Distance:%f\n",distanceToEnemy);
    printf("Turn Angle:%f\n",turnAngleEnemy);
    if(turnAngleEnemy>0.01)turn(angleRad+turnAngleEnemy);
}
double RobotBase::getDistanceToEnemy(geometry_msgs::Pose &attackPose) {
    double startX=getModelPosition().position.x;
    double startY=getModelPosition().position.y;
    double tX = attackPose.position.x;
    double tY = attackPose.position.y;
    double distance=sqrt(pow(tX-startX,2)+pow(tY-startY,2));
    return distance;
}

tuple<string,geometry_msgs::Pose, double,double> RobotBase:: findEnemy(tf::TransformListener &listener,
                                                                       tf::StampedTransform &transform){
    string enemyName;
    geometry_msgs::Pose attackPose;
    double distanceToEnemy;
    double turnAngleEnemy;

    geometry_msgs::Pose minAttackPose;
    double minDistance=-1;
    double minTurnAngleEnemy;
    auto it = enemiesNameVector.begin();
    printf("Begin_Enemy_Info:\n");
    tuple<geometry_msgs::Pose,double,double> *geometricEnemyInformation;

    while (it != enemiesNameVector.end()){
        geometricEnemyInformation=  getGeometricEnemyInformation(listener,transform,*it);
        if(geometricEnemyInformation==NULL)continue;

        attackPose= get<0>(*geometricEnemyInformation);
        distanceToEnemy= get<1>(*geometricEnemyInformation);
        turnAngleEnemy= get<2>(*geometricEnemyInformation);

        if(minDistance==-1){
            minDistance=distanceToEnemy;
            enemyName = *it;
            minAttackPose=attackPose;
            minTurnAngleEnemy = turnAngleEnemy;
        }else if(distanceToEnemy<minDistance) {
            minDistance = distanceToEnemy;
            enemyName = *it;
            minAttackPose=attackPose;
            minTurnAngleEnemy = turnAngleEnemy;
        }
        it++;

        }
    printf("End_Enemy_Info:%d\n",enemiesNameVector.size());
    return tuple<string,geometry_msgs::Pose, double,double>(enemyName,minAttackPose,minDistance,minTurnAngleEnemy);
}

void RobotBase::makeCommand(const message::CommandInformation &commandInformation){
    switch (commandInformation.commandId){
        case CM_ATTACK:
            cout<<"GET_ATTACK"<<endl;
            commandId=CM_ATTACK;
            break;
        case CM_MOVE:
            cout<<"GET_MOVE"<<endl;
            movePoseFromCommand.position.x=commandInformation.actionPoseX;
            movePoseFromCommand.position.y=commandInformation.actionPoseY;
            commandId=CM_MOVE;
            break;
    }
}

void RobotBase::live(){
    int cm=CM_NO_COMMAND;
    string enemyName;
    geometry_msgs::Pose attackPose;
    double distanceToEnemy;
    double turnAngleEnemy;
    geometry_msgs::Pose &robotPose=getModelPosition();
    tf::TransformListener listener;
    tuple<string,geometry_msgs::Pose, double,double> enemyInformation;
    double turnAngleRobot;
    try{
    while(ros::ok&&cm!=CM_ATTACK){//TODO доработать выход из цикла
        broadcastPosition();
        tf::StampedTransform transform;
        switch (commandId) {
            case CM_ATTACK:
                if(enemiesNameVector.size()>0) {
                    enemyInformation = findEnemy(listener, transform);
                    enemyName = get<0>(enemyInformation);
                    attackPose = get<1>(enemyInformation);
                    distanceToEnemy = get<2>(enemyInformation);
                    turnAngleEnemy = get<3>(enemyInformation);
                    turnAngleRobot = getAngle(getModelPosition().orientation);

                       printf("Enemy_Information_1:%d\n",enemiesNameVector.size());
                       printf("Attacker:%s\n",getModelName().data());
                       printf("Enemy Name:%s\n",enemyName.data());
                       printf("Robot Pose:X=%f Y=%f\n",robotPose.position.x,robotPose.position.y);
                       printf("Enemy Pose:X=%f Y=%f\n",attackPose.position.x,attackPose.position.y);
                       printf("Enemy Distance:%f\n",distanceToEnemy);
                       printf("Robot Turn Angle :%f\n",turnAngleRobot);
                       printf("Turn Angle:%f\n",turnAngleEnemy);
                       printf("Enemy Turn Angle:%f\n",turnAngleRobot+turnAngleEnemy);

                    if (fabs(turnAngleEnemy) > 0.1)
                        turn(turnAngleRobot + turnAngleEnemy);
                    printf("(%s)DistanceToEnemy:%f\n",getModelName().data(),distanceToEnemy);
                    if (distanceToEnemy > range) {
                        moveByAngle(listener, transform, enemyName, distanceToEnemy,
                                    turnAngleRobot + turnAngleEnemy);//TODO отрегулировать передачу дистанции
                        printf("Enemy Distance:%f Range:%d\n",distanceToEnemy,range);
                    }
                    attack(enemyName, distanceToEnemy);
                }
                break;
            case CM_MOVE:               break;
        }
        boost::this_thread::interruption_point();
    }
} catch (boost::thread_interrupted&) {
        std::cout << "INTERRUPTED" << std::endl;

        Rate rate(2);
        std_msgs::String msg;
        msg.data = getModelName();
        cout << "Delete_MSG:" << msg << endl;
        publisherDeleteItSelf.publish(msg);
        rate.sleep();
        ros::spinOnce();
    }
}

mutex sf1;
void RobotBase::checkDamage(const geometry_msgs::Pose &attackPose) {

//    sf1.lock();
    cout << "BEGIN_CHECK_DAMAGE:"<<getModelName()<<" strength:"<<strength<< endl;

if(strength<0||strength==0){
    return;
}
    if(fabs(attackPose.position.x-getModelPosition().position.x)<=0.4&&
       fabs (attackPose.position.y-getModelPosition().position.y)<=0.4){
        cout<<"DAMAGE"<<endl;
        printf("Robot_Pose:%f %f Damage_Pose:%f %f\n",
               getModelPosition().position.x,getModelPosition().position.y,
               attackPose.position.x,attackPose.position.y );
        decreaseStrength();
        if(!isLive()){
            Rate rate(2);
            cout<<"DEAD"<<endl;
            std_msgs::String msg;
            msg.data=getModelName();
            publisherKill.publish(msg);
            rate.sleep();
            ros::spinOnce();
        } else{
            cout<<"LIVE"<<endl;
         //   ros::spinOnce();
        }
    }else {
        cout<<"NO_DAMAGE"<<endl;
        printf("Robot_Pose:%f %f Damage_Pose:%f %f\n",
               attackPose.position.x,attackPose.position.y,
               getModelPosition().position.x,getModelPosition().position.y);
        // ros::spinOnce();
    }
    cout << "END_CHECK_DEMAGE:"<< endl;
   // sf1.unlock();

}


double RobotBase::getAngle(geometry_msgs::Quaternion quaternionMsg){
    tf::Quaternion quaternionTf;
    tf::quaternionMsgToTF(quaternionMsg,quaternionTf);
    return quaternionTf.getAngle();
}


geometry_msgs::Quaternion RobotBase::getQuaternionMessageByZ(double angleRad){
    tf::Quaternion q(tf::Vector3(0, 0, 1), angleRad);
    geometry_msgs::Quaternion odom;
    tf::quaternionTFToMsg(q, odom);
    return odom;
}

geometry_msgs::Quaternion RobotBase::getQuaternionMessageByX(double angleRad){
    tf::Quaternion q(tf::Vector3(0, 0, 1), angleRad);
    geometry_msgs::Quaternion odom;
    tf::quaternionTFToMsg(q, odom);
    return odom;
}

void RobotBase::changePoseByAxis(double step, AxisName axisName) {
    if (axisName==AXIS_X){
        getModelPosition().position.x=step;
    }else {
        getModelPosition().position.y = step;
    }
    setModelPosition(getModelPosition());
}

void RobotBase::turnByAxis(geometry_msgs::Pose poseDirection, AxisName axisName) {
    double directCord;
    double currentCord;
    double directionAngle;
    if (axisName==AXIS_X){
        directCord=poseDirection.position.x;
        currentCord=getModelPosition().position.x;

        if(directCord<currentCord){
            directionAngle=M_PI;
        } else directionAngle=0;
    }else {
        directCord = poseDirection.position.y;
        currentCord=getModelPosition().position.y;
        if(directCord<currentCord){
            directionAngle=-M_PI_2;
        } else directionAngle=M_PI_2;
    }
    turn(directionAngle);
}


void RobotBase::turn(double directionAngle) {
    ros::Rate rate(300);

    double delta;
    double currentAngleRadRobot;

    geometry_msgs::Pose robotPosition=getModelPosition();

    string robotName=getModelName();

    currentAngleRadRobot=getAngle(robotPosition.orientation);

    if (currentAngleRadRobot<directionAngle){
        delta=0.001;
    }else delta=-0.001;
    for (double i = currentAngleRadRobot;fabs(i-directionAngle)>0.001 ;i+=delta) {
        robotPosition.orientation= getQuaternionMessageByZ(i);
        setModelPosition(robotPosition);
        publishModelPosition(robotName,robotPosition);
        rate.sleep();
        ros::spinOnce();
        boost::this_thread::interruption_point();
    }
}

void RobotBase::moveByAxis(geometry_msgs::Pose poseDirection, AxisName axisName) {
    double delta;
    Rate rate(300);
    double directCord;
    double currentCord;
    if (axisName==AXIS_X){
        directCord=poseDirection.position.x;
        currentCord=getModelPosition().position.x;
    }
    else {
        directCord = poseDirection.position.y;
        currentCord=getModelPosition().position.y;
    }

    string robotName=getModelName();

    if (directCord<currentCord){
        delta=-0.001;
    }else delta=0.001;

    for(double step=currentCord; fabs(directCord-step)>0.1 ;step+=delta){
        changePoseByAxis(step,axisName);
        publishModelPosition(robotName,getModelPosition());
        rate.sleep();
        ros::spinOnce();
        boost::this_thread::interruption_point();
    }
}

void RobotBase::move(geometry_msgs::Pose poseDirection) {
    turnByAxis(poseDirection,AXIS_X);
    moveByAxis(poseDirection, AXIS_X);

    turnByAxis(poseDirection,AXIS_Y);
    moveByAxis(poseDirection,AXIS_Y);
}

RobotBase::~RobotBase() {
    deleteModel.request.model_name = getModelName();
    deleteModelClient.call(deleteModel);
    printf("DESTR_Base\n");
}