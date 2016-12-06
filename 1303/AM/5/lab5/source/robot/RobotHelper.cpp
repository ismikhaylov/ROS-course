//
// Created by maksim on 13.11.16.
//

#include "../../heder/robot/RobotHelper.h"

RobotHelper::RobotHelper(){}
RobotHelper::RobotHelper(NodeHandle nodeHandle,string robotName,geometry_msgs::Pose startPosition):
RobotBase(nodeHandle,robotName,startPosition){
    subscriberHelper=nodeHandle.subscribe("help_massage",10,&RobotHelper::findRobotLost,this);
}
void RobotHelper::findRobotLost(const std_msgs::String &msgHelp) {
    double endX, endY, offsetX = 1, offsetY = 1;
    publisherHelper=getNodeHandle().advertise<std_msgs::String>("directions",10);

    //std_msgs::String robotLostName = msgHelp;
    robotLostName=msgHelp;
    std_msgs::String robotHelperName;
    robotHelperName.data = getRobotName();

    gazebo_msgs::ModelState msg;
    msg.model_name = robotHelperName.data;
    msg.pose.position.x = getCurrentPosition().position.x;
    msg.pose.position.y = getCurrentPosition().position.y;

    ros::Rate r(20);
    tf::TransformListener listener;

//listener.waitForTransform("r2", "r1", ros::Time(0), ros::Duration(1));
    double tX, tY;
    while (ros::ok()) {
        broadcastPosition(msg);
        sleep(2);
        tf::StampedTransform transform;
        try {
            listener.lookupTransform(robotHelperName.data, robotLostName.data,
                                     ros::Time(0), transform);
            ROS_INFO("Transform2: %f %f", tX, tY);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        tX = transform.getOrigin().getX();
        tY = transform.getOrigin().getY();
        endX = msg.pose.position.x + tX;//5
        endY = msg.pose.position.y + tY;//5

        msg.pose.position.x += offsetX;
        msg.pose.position.y += offsetY;

        if (msg.pose.position.x <= endX - 1 && msg.pose.position.y <= endY - 1) {
            setCurrentPosition(msg.pose);
            getPublisher().publish(msg);
            broadcastPosition(msg);
            r.sleep();
        } else {
            ros::Rate rate(2);
            publisherHelper.publish(robotHelperName);
            rate.sleep();
            publisherHelper.publish(robotHelperName);
            goBack();
            return;
        }
    }
}

void  RobotHelper::goBack() {
    double  offsetX = 1, offsetY = 1;
    //publisherHelper=getNodeHandle().advertise<std_msgs::String>("directions",10);

  //  std_msgs::String robotLostName = msgHelp;
    std_msgs::String robotHelperName;
    robotHelperName.data = getRobotName();
double xt=4,yt=4;
    gazebo_msgs::ModelState msg;
    msg.model_name = robotHelperName.data;
    msg.pose.position.x = getCurrentPosition().position.x;
    msg.pose.position.y = getCurrentPosition().position.y;

    //msg.pose.position.x = xt;
    //msg.pose.position.y = yt;

    ros::Rate r(20);
    tf::TransformListener listener;

//listener.waitForTransform("r2", "r1", ros::Time(0), ros::Duration(1));
    double tX, tY;
    while (ros::ok()) {
        broadcastPosition(msg);
        sleep(2);
        tf::StampedTransform transform;
        try {
            listener.lookupTransform(robotHelperName.data, robotLostName.data,
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        tX=transform.getOrigin().getX();
        tY=transform.getOrigin().getY();

        ROS_INFO("Transform2: %f %f", tX, tY);

        msg.pose.position.x-=offsetX;
        msg.pose.position.y-=offsetY;
        if (msg.pose.position.x >= endX && msg.pose.position.y >= endY) {
            setCurrentPosition(msg.pose);
            getPublisher().publish(msg);
            broadcastPosition(msg);

        } else return;
    }
}