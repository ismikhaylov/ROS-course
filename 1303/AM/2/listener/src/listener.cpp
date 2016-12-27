#include "ros/ros.h"
#include "std_msgs/String.h"
#include </home/maksim/ros_kinetic/workspace_lab2/devel/include/message/TargetCoordinates.h>
#include "vector"
using namespace std;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%

vector<double*> createAntiRocket(int countAntiRockets,int borderSpaceX,int borderSpaceY,int borderSpaceZ){
    vector<double*> antiRockets;
    for(int i=0;i<countAntiRockets;i++){
        double *coordinatesAntiRocket=new double [3];
        coordinatesAntiRocket[0]=rand()%(borderSpaceX+1);
        coordinatesAntiRocket[1]=rand()%(borderSpaceY+1);
        coordinatesAntiRocket[2]=rand()%(borderSpaceZ+1);
        antiRockets.push_back(coordinatesAntiRocket);
    }
    return antiRockets;
}

bool rocketDestroy(int chanceDestroy){
    bool isDestroy=false;
    int val=rand()%100+1;
    if(val<=chanceDestroy)isDestroy=true;
    return isDestroy;
}

void rocketIntercaption(const message::TargetCoordinates::ConstPtr &targetCoordinates)
{
    int chanceDetection=80;
    int chanceNotDetection=5;
    int countAntiRocket=0;
    int numberAntiRocket=-1;
    double *coordinatesAntiRocket;
    vector<double*> antiRockets;
    antiRockets=   createAntiRocket(3,1,1,1);
    vector<double*>::iterator itBegin=antiRockets.begin();
    vector<double*>::iterator itEnd=antiRockets.end();
    bool rocketDetection=false;
    ROS_INFO("--------BEGIN INTERCEPTION");
    while (itBegin!=itEnd&&!rocketDetection) {
        coordinatesAntiRocket= (*itBegin);

        if(coordinatesAntiRocket[0]==targetCoordinates->x &&
                coordinatesAntiRocket[1]==targetCoordinates->y &&
                coordinatesAntiRocket[2]==targetCoordinates->z){
            rocketDetection=true;
            numberAntiRocket=countAntiRocket;
        }
        countAntiRocket++;
        itBegin++;
    }

    if(rocketDetection){
       coordinatesAntiRocket= antiRockets.at(numberAntiRocket);
        ROS_INFO("Rocket has detected by antiRocket with number %d and\n"
                 "coordinates: %f %f %f",numberAntiRocket,coordinatesAntiRocket[0],coordinatesAntiRocket[1],coordinatesAntiRocket[2]);
    }
    else ROS_INFO("Rocket has not detected by antiRockets");

    if(rocketDetection)
    {   if(rocketDestroy(chanceDetection)) ROS_INFO("Rocket has destroyed with chance %d",chanceDetection);
        else ROS_INFO("Rocket has not destroyed with chance %d",chanceDetection);
    }
    else{ if(rocketDestroy(chanceNotDetection)) ROS_INFO("Rocket has destroyed with chance %d",chanceNotDetection);
        else ROS_INFO("Rocket has not destroyed with chance %d",chanceNotDetection);
    }
    ROS_INFO("--------END INTERCEPTION\n\n");
}
// %EndTag(CALLBACK)%



void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;


    ros::Subscriber sub = n.subscribe("/coordinate_space", 1000,rocketIntercaption);
    // %EndTag(SUBSCRIBER)%

    // %Tag(SPIN)%
    ros::spin();
    // %EndTag(SPIN)%

    return 0;
}

