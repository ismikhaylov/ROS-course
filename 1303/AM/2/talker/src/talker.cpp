#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <ctime>
#include </home/maksim/ros_kinetic/workspace_lab2/devel/include/message/TargetCoordinates.h>

void rocketFiring(int countRocket,int borderSpaceX,int borderSpaceY,int borderSpaceZ){
    ros::NodeHandle n;
    ros::Publisher topic=n.advertise<message::TargetCoordinates>("/coordinate_space",1000);
    ros::Rate loop_rate(10);
    message::TargetCoordinates targetPosition;
    int i;
for(i=0;i<countRocket;i++)
      {
    targetPosition.x=rand()%(borderSpaceX+1);
    targetPosition.y=rand()%(borderSpaceY+1);
    targetPosition.z=rand()%(borderSpaceZ+1);
    ROS_INFO("Fired rocket to position:\n"
              "x= %f y= %f z= %f\n",
             targetPosition.x,targetPosition.y,targetPosition.z);
    topic.publish(targetPosition);
    loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  rocketFiring(10,1,1,1);
  return 0;
}
