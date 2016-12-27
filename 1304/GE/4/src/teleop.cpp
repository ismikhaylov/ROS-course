#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <cmath>

#define PI 3.14159265

float target_x = 7;
float target_y = 6;

float robot_x;
float robot_y;
float robot_angle = 0;

float step = 1.0;

ros::Publisher movement_publisher;

bool finish = false;

bool forwardWall = false;
bool rightWall = false;

void laserScanCallback(const sensor_msgs::LaserScan& msg)
{
  forwardWall = false;
  rightWall = false;

  for (int i = 0; i < 2 * msg.ranges.size() / 9; i++)
  {
    if (msg.ranges[i] < step)
    {
      rightWall = true;
      break;
    }
  }
  for (int i = 3 * msg.ranges.size() / 9; i < 5 * msg.ranges.size() / 9; i++)
  {
    if (msg.ranges[i] < step)
    {
      forwardWall = true;
      break;
    }
  }
}

void checkFinish()
{
  finish = (pow(robot_x - target_x,2) + pow(robot_y - target_y, 2)) <= pow(step/2,2);
}

void odometryCallback(const nav_msgs::Odometry& msg)
{
  robot_x = msg.pose.pose.position.x;
  robot_y = msg.pose.pose.position.y;  
  robot_angle = msg.pose.pose.orientation.z;  
  checkFinish();
}

void goAhead()
{
  geometry_msgs::Twist m_msg;     
  m_msg.linear.x = step;
  movement_publisher.publish(m_msg);
}

void rotate(float angle)
{
  geometry_msgs::Twist m_msg;     
  m_msg.angular.z = angle;
  movement_publisher.publish(m_msg);
}

void goToTarget()
{
  static bool rotated = false;
  if (rotated && !rightWall)
  {
    rotate(-0.1 * PI);
  } 
  else if(robot_x != target_x && !forwardWall)
  {
    goAhead();
  } 
  else if (forwardWall)
  {
    rotate(0.1*PI);
    rotated = true;
  } 
  else if (!rotated)
  {
    rotate(0.1*PI);
    rotated = true;
  } 
  else if (robot_y != target_y && !forwardWall)
  {
    goAhead();
  } 
  else if(forwardWall)
  {
    rotate(0.1*PI);
    rotated = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop");

  ros::NodeHandle n;
  movement_publisher = n.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1000);
  ros::Subscriber laser_subscriber = n.subscribe("/robot_0/base_scan", 1000, laserScanCallback);
  ros::Subscriber robot_subscriber = n.subscribe("/robot_0/base_pose_ground_truth", 1000, odometryCallback);
  ros::Rate rate(5);

  while(ros::ok() && !finish)
  {
    goToTarget();
    checkFinish();
    ros::spinOnce();
    rate.sleep();
  } 
  return 0;
}