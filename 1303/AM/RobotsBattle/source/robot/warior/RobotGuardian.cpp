//
// Created by maksim on 16.11.16.
//

#include "../../../heder/robot/warior/RobotGuardian.h"
RobotGuardian::RobotGuardian() {}
RobotGuardian::RobotGuardian(NodeHandle nodeHandle,string robotName,geometry_msgs::Pose startPosition,
                             geometry_msgs::Pose swordPosition,vector<string>enemiesName):
        RobotBase(nodeHandle,string(PATH_TO_USER_DIRECTORY)+string(PATH_TO_ROBOT_GUARDIAN_MODEL),robotName,startPosition,enemiesName){
    sword=new Sword(nodeHandle,string(PATH_TO_USER_DIRECTORY)+string(PATH_TO_ROBOT_GUARDIAN_SWORD_MODEL),
                string(PREFIX_ROBOT_GUARDIAN_SWORD)+getModelName(),swordPosition,1);
    range=sword->getDistanceAttack();

    setStrength(3);

}
void RobotGuardian::attack(string &enemyName, double &distanceToEnemy) {
    //TODO Uncomment
    if(enemyName.empty()||enemiesNameVector.size()==0)
        return;

    ros::Rate rate(300);

    double delta=0.01;
    double turnAngleAttack;
    double currentAngleRadSword;

    geometry_msgs::Pose swordPosition=sword->getModelPosition();
    currentAngleRadSword=getAngle(sword->getModelPosition().orientation);
    turnAngleAttack=currentAngleRadSword-0.5;
    cout<<"Angle:"<<currentAngleRadSword<<endl;
    ros::Rate rateMove(2);

    for (double i = currentAngleRadSword;i>turnAngleAttack ;i-=delta) {

        swordPosition.orientation= getQuaternionMessageByZ(i);

        sword->getModelPosition()=swordPosition;

        sword->publishModelPosition(sword->getModelName(),swordPosition);

        rate.sleep();
        ros::spinOnce();
        boost::this_thread::interruption_point();
    }

    geometry_msgs::Pose robotPosition = getModelPosition();
    double angleRad=getAngle(robotPosition.orientation);
    geometry_msgs::Pose poseAttack;
    poseAttack.position.x = robotPosition.position.x + 1 * cos(angleRad);
    poseAttack.position.y = robotPosition.position.y + 1 * sin(angleRad);

    Publisher *publisherEnemies= publisherEnemiesMap[enemyName];

    publisherEnemies->publish(poseAttack);
    rateMove.sleep();
    ros::spinOnce();

    for (double i = turnAngleAttack;i<currentAngleRadSword;i+=delta) {

        swordPosition.orientation= getQuaternionMessageByZ(i);

        sword->getModelPosition()=swordPosition;

        sword->publishModelPosition(sword->getModelName(),swordPosition);

        rate.sleep();
        ros::spinOnce();
        boost::this_thread::interruption_point();
    }
}

