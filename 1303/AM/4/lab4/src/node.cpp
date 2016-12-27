#include "tf/LinearMath/Transform.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <nav_msgs/Odometry.h>
#include "math.h"
#include<algorithm>
#define ERROR_CREATE_THREAD -11
#define ERROR_JOIN_THREAD   -12
#define SUCCESS        0

double curentX,curentY;
double directionX,directionY;

double turnToHalfPi=0;
double turnToZero=0;
double turnToPi=0;
double turnToNegativeHalfPi=0;

double speed=2;

double minDistanceToWall=0.4;
double minDistanceToWallLeft=0.5;
double minDistanceToWallRight=0.5;


bool leftSide=false;
bool midSide=false;
bool rightSide=false;

double rateTime=2;
enum analyzeSide{SIDE_RIGHT,SIDE_LEFT};
analyzeSide analyzeEnum;
//--------

double minRight;
double minLeft;
double minMid;


int ind=0;

double limit=0.5;
double linearX=0.5;
double angle=60;
double angleInc=0.6;

double angularOneGr=0.08;

using namespace std;

void getRobotPosition(const nav_msgs::Odometry& msg) {
    
    curentX = msg.pose.pose.position.x;
    curentY = msg.pose.pose.position.y;
    tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    turnToZero=-yaw;
    turnToHalfPi=M_PI_2-yaw;
    turnToPi=M_PI-yaw;
    turnToNegativeHalfPi=(-M_PI_2-yaw);

}



void chatterCallback(const sensor_msgs::LaserScan& msg)
{

    minRight=*std::min_element(msg.ranges.begin(),msg.ranges.begin()+50);
    minLeft=*std::min_element(msg.ranges.begin()+150,msg.ranges.end());
    minMid=*std::min_element(msg.ranges.begin()+50,msg.ranges.begin()+150);

    leftSide=minLeft<minDistanceToWallLeft ? true :false;
    rightSide=minRight<minDistanceToWallRight ? true :false;
    midSide=minMid<minDistanceToWall ? true:false;

}

void turn(ros::Publisher&publisher,ros::Rate &rate, double angle){
    geometry_msgs::Twist pos;
    pos.angular.z=angle;
    publisher.publish(pos);
    rate.sleep();
    ros::spinOnce();
}

void turnToX(ros::Publisher&publisher){

    if(directionX<0){
        analyzeEnum=SIDE_LEFT;
    }else
        analyzeEnum=SIDE_RIGHT;

    ros::Rate rate(rateTime);
    double angle=directionX<0 ? turnToZero:turnToPi;
    while(fabs(angle)>0.001){
        turn(publisher,rate,angle);
        angle=directionX<0 ? turnToZero:turnToPi;
    }
}

void turnToY(ros::Publisher&publisher){

    if(directionY<0){
        analyzeEnum=SIDE_RIGHT;
    }else
        analyzeEnum=SIDE_LEFT;

    ros::Rate rate(rateTime);
    double angle=directionY<0 ? turnToHalfPi:turnToNegativeHalfPi;
    while(fabs(angle)>0.001){
        turn(publisher,rate,angle);
        angle=directionY<0 ? turnToHalfPi:turnToNegativeHalfPi;
    }
}


void move(ros::Publisher&publisher,ros::Rate rate){
    geometry_msgs::Twist pos;
    pos.linear.x=speed;
    publisher.publish(pos);
    rate.sleep();
    ros::spinOnce();
}

void moveSteps(ros::Publisher&publisher,ros::Rate rate,int numberSteps){
    geometry_msgs::Twist pos;
    pos.linear.x=speed;
    for(int i=0;i<numberSteps;i++){
    publisher.publish(pos);
    rate.sleep();
    ros::spinOnce();
    }
}


int main(int argc, char **argv)
{
    bool stopedX=false,stopedY=false;
    bool analizeSide=false;
    //----------
    double endX=4,endY=6.5;
    int numberSteps=3;
    ros::init(argc, argv, "talker");
    ros::NodeHandle node;
    ros::Publisher publisher=node.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    ros::Rate rate(rateTime);
    ros::Subscriber subscriber = node.subscribe("/base_pose_ground_truth" ,1000, getRobotPosition);
    ros::Subscriber sub = node.subscribe("/base_scan", 1000, chatterCallback);
    rate.sleep();
    ros::spinOnce();
   while(fabs(endX-curentX)>0.5||fabs(endY-curentY)>0.5)
    {
        directionX=endX<curentX ? 1 : -1 ;
        turnToX(publisher);

        while(!midSide&&(stopedY||fabs(endX-curentX)>0.5)){
            move(publisher,rate);
            analizeSide=analyzeEnum==SIDE_LEFT? leftSide:rightSide;
           if(stopedY&&!analizeSide){
                stopedY=false;
                moveSteps(publisher,rate,numberSteps);
                break;
            }
        }

        rate.sleep();
        ros::spinOnce();
        stopedX=midSide;
        directionY=endY<curentY ? 1 : -1;
        turnToY(publisher);
        while(!midSide&&(stopedX||fabs(endY-curentY)>0.5)){
            move(publisher,rate);
            analizeSide=analyzeEnum==SIDE_LEFT? leftSide:rightSide;
            if(stopedX&&!analizeSide){
                stopedX=false;
                moveSteps(publisher,rate,numberSteps);
                break;
            }
        }
        stopedY=midSide;
    }
    return 0;
}
