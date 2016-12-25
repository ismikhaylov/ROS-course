//
// Created by maksim on 16.11.16.
//

#include "../../../heder/robot/warior/RobotArcher.h"

RobotArcher::RobotArcher() {}

RobotArcher::RobotArcher(NodeHandle nodeHandle,string robotName,geometry_msgs::Pose startPosition,
                         geometry_msgs::Pose bowPosition,vector<string>enemiesName):
RobotBase(nodeHandle,string(PATH_TO_USER_DIRECTORY)+string(PATH_TO_ROBOT_ARCHER_MODEL),robotName,startPosition,enemiesName){
    bow=new Bow(nodeHandle,string(PATH_TO_USER_DIRECTORY)+string(PATH_TO_ROBOT_ARCHER_BOW_MODEL),
                string(PREFIX_ROBOT_ARCHER_BOW)+getModelName(),bowPosition,5);
    range=bow->getDistanceAttack();
}

mutex sf;

void RobotArcher::setModelPosition(geometry_msgs::Pose &pose){
    pose.position.z=poseRobotByZ;//TODO
    RobotBase::setModelPosition(pose);

    double angle= getAngle(pose.orientation);
    angle+=M_PI;
    geometry_msgs::Pose pose1=pose;
    pose1.orientation= getQuaternionMessageByZ(angle);
    pose1.position.z=poseBowByZ;
    bow->setModelPosition(pose1);
    bow->getArrow()->setModelPosition(pose1);
}

void RobotArcher::publishModelPosition(string modelName,geometry_msgs::Pose modelPosition) {
    //cout<<"PUBLISH_ARCHER"<<endl;
    RobotBase::publishModelPosition(modelName,modelPosition);

    gazebo_msgs::ModelState modelState;
    modelState.model_name =bow->getModelName();
    modelState.pose=bow->getModelPosition();
    getPublisher().publish(modelState);

    modelState.model_name =bow->getArrow()->getModelName();
    modelState.pose=bow->getArrow()->getModelPosition();
    getPublisher().publish(modelState);

}

void RobotArcher::changePoseByAxis(double step, AxisName axisName) {
    RobotBase::changePoseByAxis(step,axisName);
    if (axisName==AXIS_X){
        bow->getModelPosition().position.x=step;
        bow->getArrow()->getModelPosition().position.x=step;
    }else {
        bow->getModelPosition().position.y = step;
        bow->getArrow()->getModelPosition().position.y = step;
    }
    setModelPosition(getModelPosition());
}


//TODO добавит проверку - можно ли стрелять(добавить listener и transform)
void RobotArcher::attack(string &enemyName,double &distanceToEnemy) {
    if(enemyName.empty()||enemiesNameVector.size()==0)
        return;

    sf.lock();
    ros::Rate rateAttack(300);
    double delta = 0.01;
    int distanceAttack = bow->getDistanceAttack();

    if(distanceToEnemy<distanceAttack)
       distanceAttack = distanceToEnemy;
    Arrow *arrow = bow->getArrow();

    geometry_msgs::Pose robotPosition = getModelPosition();

    gazebo_msgs::ModelState modeState;
    modeState.model_name = arrow->getModelName();
    modeState.pose = arrow->getModelPosition();

    geometry_msgs::Pose pose;
    pose.position.z = 0.35;

    double angleRad = getAngle(robotPosition.orientation);
    double minAngleRad=angleRad-0.4;
    double maxAngleRad=angleRad+0.4;
    //angleRad= minAngleRad+2*random;
    double randomAngleRad = (rand()/(double)RAND_MAX)* (maxAngleRad - minAngleRad) + minAngleRad;
    printf("%f:Attack(%s)-->%s:%f %d\n",randomAngleRad,getModelName().data(),enemyName.data(),distanceToEnemy,distanceAttack);
    angleRad=randomAngleRad;
    double j;
    for ( j = 0; j < distanceToEnemy ; j += delta) {
        ///***** TRUE
        pose.position.x = robotPosition.position.x + j * cos(angleRad);
        pose.position.y = robotPosition.position.y + j * sin(angleRad);

      //  cout<<getModelName()<<" "<<"Delta:"<<j<<endl;

        arrow->setModelPosition(pose);
        arrow->publishModelPosition(arrow->getModelName(), arrow->getModelPosition());

        rateAttack.sleep();
        ros::spinOnce();
        boost::this_thread::interruption_point();
    }
    geometry_msgs::Pose poseAttack = pose;

    ros::Rate rateMove(2);
   // rateMove.sleep();

    pose.position.x = robotPosition.position.x;
    pose.position.y = robotPosition.position.y;

    arrow->setModelPosition(pose);

    for (int i = 0; i < 1; i++){
    arrow->publishModelPosition(arrow->getModelName(), pose);
    rateMove.sleep();
    ros::spinOnce();
    }

    Publisher *publisherEnemies= publisherEnemiesMap[enemyName];

    publisherEnemies->publish(poseAttack);
    rateMove.sleep();
    ros::spinOnce();
    sf.unlock();
}


RobotArcher::~RobotArcher(){

    deleteModel.request.model_name =bow->getModelName();
    deleteModelClient.call(deleteModel);

    deleteModel.request.model_name =bow->getArrow()->getModelName();
    deleteModelClient.call(deleteModel);
    printf("DESTR_ARCHER\n");

}

geometry_msgs::Pose RobotArcher::getAttackPosition(double angleRad, int step){
    geometry_msgs::Pose &robotPosition=getModelPosition();
    geometry_msgs::Pose attackPosition;

    attackPosition.position.x=robotPosition.position.x+step*cos(angleRad);
    attackPosition.position.y=robotPosition.position.y+step*sin(angleRad);
    attackPosition.position.z=0.35;

    return attackPosition;
}
