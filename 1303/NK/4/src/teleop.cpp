#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

float target_x = 7;
float target_y = 6;

float robot_x;
float robot_y;
float robot_angle = 0;

float speed = 0.5;

ros::Publisher velocity_publisher;

bool reach = false;
bool rotated = false;

bool forwardWall = false;
bool rightWall = false;

float delta = 0.5;

#define PI 3.14159265

void updatePosFlags(){
  reach = (pow(robot_x - target_x,2) + pow(robot_y - target_y, 2)) <= pow(delta,2);
}

void updateWallInfo(const sensor_msgs::LaserScan& msg) {
  forwardWall = false;
  rightWall = false;

  for (int i = 0; i < 2*msg.ranges.size()/9; i++) {
    if (msg.ranges[i] < 2*speed) {
      rightWall = true;
      break;
    }
  }
  for (int i = 3*msg.ranges.size() / 9; i < 5*msg.ranges.size() / 9; i++) {
    if (msg.ranges[i] < 2*speed) {
      forwardWall = true;
      break;
    }
  }
}

void updateRobotPos(const nav_msgs::Odometry& msg) {
  robot_x = msg.pose.pose.position.x;
  robot_y = msg.pose.pose.position.y;  
  robot_angle = msg.pose.pose.orientation.z;  
  updatePosFlags();
}

void goForward(){
  //ROS_INFO_STREAM("goForward()");
  geometry_msgs::Twist vel_msg;     
  vel_msg.linear.x = speed;
  velocity_publisher.publish(vel_msg);
}

void rotate(float angle){
  //ROS_INFO_STREAM("rotate()");
  //printf("angle = %f;\n", angle);
  geometry_msgs::Twist vel_msg;     
  vel_msg.angular.z = angle;
  velocity_publisher.publish(vel_msg);
}

void goToTarget(){
  //printf("rotated = %d; robot_angle = %f;\nforwardWall = %d; rightWall = %d;\nrobot_x = %f; robot_y = %f;\ntarget_x = %f; target_y = %f;\n", rotated, robot_angle, forwardWall, rightWall, robot_x, robot_y, target_x, target_y);

  if (rotated && !rightWall){
    rotate(-0.1*PI);
  } else if(robot_x != target_x && !forwardWall){
    goForward();
  } else if (forwardWall){
    rotate(0.1*PI);
    rotated = true;
  } else if (!rotated) {
    rotate(0.1*PI);
    rotated = true;
  } else if (robot_y != target_y && !forwardWall){
    goForward();
  } else if(forwardWall){
    rotate(0.1*PI);
    rotated = true;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop");

  ros::NodeHandle n;
  velocity_publisher = n.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1000);
  ros::Subscriber laser_subscriber = n.subscribe("/robot_0/base_scan", 1000, updateWallInfo);
  ros::Subscriber robot_subscriber = n.subscribe("/robot_0/base_pose_ground_truth", 1000, updateRobotPos);
  ros::Rate rate(5);

  while(ros::ok() && !reach){
    goToTarget();
    updatePosFlags();
    ros::spinOnce();
    rate.sleep();
  } 
  return 0;
}