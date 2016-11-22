//
// Created by maksim on 13.11.16.
//

#include "../../heder/robot/RobotLost.h"

RobotLost::RobotLost(){

}


RobotLost::RobotLost(NodeHandle nodeHandle,string robotName,geometry_msgs::Pose currentPosition):
RobotBase(nodeHandle,robotName,currentPosition){
    publisherHelpMassage=nodeHandle.advertise<std_msgs::String>("help_massage",10);
    subscriberDirectionsMassage=nodeHandle.subscribe("directions",10,&RobotLost::performDirections,this);
    isHelp= false;
}

void RobotLost::performDirections(const std_msgs::String &msgDirections) {
    if(isHelp)  return;
    ROS_INFO("*****performDirections");
    cout<<msgDirections<<endl;

    double offsetX = 1, offsetY = 1;

    std_msgs::String robotLostName ;
    std_msgs::String robotHelperName=msgDirections;
    robotLostName.data = getRobotName();


    gazebo_msgs::ModelState msg;
    msg.model_name = robotLostName.data;
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
            listener.lookupTransform(robotLostName.data, robotHelperName.data,
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        tX = transform.getOrigin().getX();
        tY = transform.getOrigin().getY();
        double realX, realY;

        modf(tX, &realX);
        modf(tY, &realY);


        ROS_INFO("Transform1: %f %f", realX, realY);

        msg.pose.position.x += realX;
        msg.pose.position.y += realY;
        if (msg.pose.position.x >= endX+1 && msg.pose.position.y >= endY+1) {
            getPublisher().publish(msg);
            broadcastPosition(msg);
        }else {
            isHelp = true;
            cout<<"END"<<endl;
            return;
        }
    }

}

void RobotLost::m1(){
    gazebo_msgs::ModelState msg;
    msg.model_name = getRobotName();
    msg.pose.position.x=getCurrentPosition().position.x;
    msg.pose.position.y=getCurrentPosition().position.y;
    Publisher publisher=getPublisher();
    ros::Rate r(20);
    int count=0;
    while(ros::ok()){
        publisher.publish(msg);
        broadcastPosition(msg);
        count++;
        r.sleep();
        ros::spinOnce();
        sleep(2);
        if(isHelp)return;
    }
}

void RobotLost::sendHelpMassage() {
    ros::Rate rate(2);
    std_msgs::String robotName;
    robotName.data=getRobotName();
    publisherHelpMassage.publish(robotName);
    rate.sleep();
    publisherHelpMassage.publish(robotName);
}