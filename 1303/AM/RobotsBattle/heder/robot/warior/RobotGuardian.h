//
// Created by maksim on 16.11.16.
//

#ifndef ROBOTSBATTLE_ROBOTGUARDIAN_H
#define ROBOTSBATTLE_ROBOTGUARDIAN_H

#define PATH_TO_ROBOT_GUARDIAN_MODEL "/.gazebo/models/husky/model.sdf"
#define PATH_TO_ROBOT_GUARDIAN_SWORD_MODEL "/.gazebo/models/drc_practice_2x6/model.sdf"
#define PREFIX_ROBOT_GUARDIAN_SWORD "sword_"

#include "../base/RobotBase.h"
#include "../item/weapon/Sword.h"

class RobotGuardian :public RobotBase{
    Sword *sword;
    const double poseRobotByZ=0.15;
public:
    RobotGuardian();
    RobotGuardian(NodeHandle nodeHandle,string robotName,geometry_msgs::Pose startPosition,
                  geometry_msgs::Pose swordPosition,vector<string>enemiesName);

public:
    virtual void attack(string &enemyName,double &distanceToEnemy);
    virtual void hideRobotModel(){
        geometry_msgs::Pose &pose=getModelPosition();
        pose.position.z-=0.5;
        Rate rate(30);
            for(int i;i<50;i++)
        {
            RobotBase::setModelPosition(pose);
            sword->setModelPosition(pose);

            publishModelPosition(getModelName(),pose);
            rate.sleep();
            ros::spinOnce();
        }


    }

    virtual void setModelPosition(geometry_msgs::Pose &pose){
        pose.position.z=poseRobotByZ;//TODO
        RobotBase::setModelPosition(pose);

        double turnAngleSword=getAngle(pose.orientation);

        tf::Quaternion q1(tf::Vector3(0, 0,1 ), turnAngleSword);
        geometry_msgs::Quaternion odom1;
        tf::quaternionTFToMsg(q1, odom1);

        geometry_msgs::Pose &swordPose=sword->getModelPosition();

        swordPose.position.x = getModelPosition().position.x + 0.45 * cos(getAngle(pose.orientation));
        swordPose.position.y = getModelPosition().position.y + 0.45 * sin(getAngle(pose.orientation));

        swordPose.orientation=odom1;
        sword->setModelPosition(swordPose);

    }

    virtual void publishModelPosition(string modelName,geometry_msgs::Pose modelPosition) {
        RobotBase::publishModelPosition(modelName,modelPosition);
        gazebo_msgs::ModelState modelState;
        modelState.model_name =sword->getModelName();
        modelState.pose=sword->getModelPosition();
        getPublisher().publish(modelState);
    }
};


#endif //ROBOTSBATTLE_ROBOTGUARDIAN_H
