#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include "math.h"
void waiting(ros::Publisher pub,int time){
    while (pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return ;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(time);
    }
    
}

void drawRobot(){
    ros::NodeHandle nh;
    ros::Publisher pub =
            nh.advertise<visualization_msgs::Marker>("visualization_marker1",1);
    double startX=5;
    double startY=5;
    double startZ=0.5;
    ros::Rate r(30);
    
    uint32_t shape = visualization_msgs::Marker::CUBE;
    
    double offset = 0;
    visualization_msgs::Marker msg;
    msg.header.frame_id = "/my_frame";
    msg.header.stamp = ros::Time::now();
    msg.ns = "robot1";
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.position.x = 5;
    msg.pose.position.y = 5;
    msg.pose.position.z = 0.5;
    
    msg.id = 0;
    msg.type = shape;
    
    msg.scale.x = 1.0;
    msg.scale.y = 1.0;
    msg.scale.z = 1.0;
    
    msg.color.r = 1.0f;
    msg.color.g = 0.0f;
    msg.color.b = 0.0f;
    msg.color.a = 5.0;
    
    int count=0;
    tf::TransformListener listener;
    double realX,realY;
    bool isHelp=false;
double R=5,x0=5,y0=5,con=2,f;
double alpha = con * M_PI / 180;
    while(ros::ok())
    {
        waiting(pub,1);
sleep(2);
        tf::StampedTransform transform;
        try{

          listener.lookupTransform("r1", "r2",
                                   ros::Time(0), transform);
         
          ROS_INFO("Transform %f %f %f",
                   transform.getOrigin().getX(),
                   transform.getOrigin().getY(),
                   transform.getOrigin().getZ());
          double tX= transform.getOrigin().getX();
          double tY=transform.getOrigin().getY();
          double tZ=transform.getOrigin().getZ();
          modf(tX,&realX);
          modf(tY,&realY);

          if(fabs(tX)<=2&&fabs(tY)<=2){
              isHelp=true;
          }
          if(isHelp){
              msg.pose.position.x = (startX+=realX);
              msg.pose.position.y = (startY+=realY);
              msg.pose.position.z = 0.5;
          }
if(msg.pose.position.x<=-7&&msg.pose.position.y<=-7)
    break;
            pub.publish(msg);
        }
        catch (tf::TransformException &ex) {
            pub.publish(msg);
          ROS_ERROR("%s",ex.what());
          ros::Duration(2.0).sleep();
          continue;
        }
        count++;
    }
    r.sleep();
}

int main(int argc, char **argv) {
    ros::init(argc,argv,"node1");
    drawRobot();
    sleep(2);
    
    return 0;
}