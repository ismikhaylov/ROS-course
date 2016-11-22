#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <string>
#include "tf/LinearMath/Transform.h"

const double Pi = 3.14159;
double toPi;
double toPartPi;
double negPartPi;
double toZero;

float dirX;
float dirY;

float curX;
float curY;
double curAngle;

bool stop = false;
bool left = false;
bool right = false;

const float speed = 0.5;

enum side {RIGHT, LEFT};
side sideForAnalyze;

void checkPath(const sensor_msgs::LaserScan& msg) {
  left = false;
  right = false;
  stop = false;
  for (int i = 3*msg.ranges.size() / 4; i < 4*msg.ranges.size() / 4; i++) {
    if (msg.ranges[i] < 0.5) {
      left = true;
    }
  }
  for (int i = 0; i < msg.ranges.size() / 4; i++) {
    if (msg.ranges[i] < 0.5) {
      right = true;
    }
  }
  for (int i = msg.ranges.size() / 4; i < 2*msg.ranges.size() / 4; i++) {
    if (msg.ranges[i] < 0.4) {
      stop = true;
      return;
    }
  }
}

void getRobotPosition(const nav_msgs::Odometry& msg) {
  curX = msg.pose.pose.position.x;
  curY = msg.pose.pose.position.y;
  tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                   msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  toPi = (Pi - yaw)*2;
  toPartPi = (Pi/2 - yaw)*2;
  negPartPi = (-Pi/2 - yaw)*2;
  toZero = -yaw*2;
  curAngle = yaw;
}

void roll(ros::Publisher &publisher, ros::Rate &rate, double angle) {
  geometry_msgs::Twist msg;
  msg.angular.z = angle;
  publisher.publish(msg);
  rate.sleep();
  ros::spinOnce();
}

void toX(ros::Publisher &publisher) {
  if (fabs(curAngle-Pi/2) < 0.001) {
    if (dirX > 0) {
      sideForAnalyze = RIGHT;
    } else {
      sideForAnalyze = LEFT;
    }
  } else  {
    if (dirX > 0) {
      sideForAnalyze = LEFT;
    } else {
      sideForAnalyze = RIGHT;
    }
  }
  ros::Rate rate(2);
  double angle = dirX > 0 ? toPi : toZero;
  while (fabs(angle) > 0.001) {
    roll(publisher, rate, angle);
    angle = dirX > 0 ? toPi : toZero;
  }
}

void toY(ros::Publisher &publisher) {
  if (fabs(curAngle-Pi) < 0.001) {
    if (dirY > 0) {
      sideForAnalyze = RIGHT;
    } else {
      sideForAnalyze = LEFT;
    }
  } else  {
    if (dirY > 0) {
      sideForAnalyze = LEFT;
    } else {
      sideForAnalyze = RIGHT;
    }
  }
  ros::Rate rate(2);
  double angle = dirY > 0 ? negPartPi : toPartPi;
  while (fabs(angle) > 0.001) {
    roll(publisher, rate, angle);
    angle = dirY > 0 ? negPartPi : toPartPi;
  }
}

void shift(ros::Publisher &publisher, ros::Rate &rate) {
  geometry_msgs::Twist msg;
  msg.linear.x = 6*speed;
  for (int i = 0; i < 3; i++) {
    publisher.publish(msg);
    rate.sleep();
    ros::spinOnce();
  }
}

void step(ros::Publisher &publisher, ros::Rate &rate) {
  geometry_msgs::Twist msg;
  msg.linear.x = speed;
  publisher.publish(msg);
  rate.sleep();
  ros::spinOnce();
}

int main(int argc, char** argv) {
  bool wasStopX = false;
  bool wasStopY = false;

  ros::init(argc, argv, "robot_mover");
  ros::NodeHandle nodeHandle;
  const int queueSize = 1000;
  if (argc < 3) {
    ROS_ERROR("It's necessary to set end position.");
    return 1;
  }
  float endX = std::stof(argv[1]);
  float endY = std::stof(argv[2]);

  ros::Publisher publisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", queueSize);

  ros::Rate rate(2);

  ros::Subscriber subscriber = nodeHandle.subscribe("/base_pose_ground_truth" , queueSize, getRobotPosition);
  ros::Subscriber scanSubscriber = nodeHandle.subscribe("/base_scan", queueSize, checkPath);
  rate.sleep();
  ros::spinOnce();
  bool changeX = false;
  bool changeY = false;

  while (fabs(curX - endX) > 0.5 || fabs(curY - endY) > 0.5) {
    float oldDirX = dirX;
    dirX = endX < curX ? 1 : -1;
    changeX = false;
    toX(publisher);
    if (wasStopX && wasStopY && oldDirX == dirX) {
      dirX*= -1;
      side oldSide = sideForAnalyze;
      toX(publisher);
      wasStopX = false;
      changeX = true;
      sideForAnalyze = oldSide == LEFT ? RIGHT : LEFT;
    }
    while (!stop && (wasStopY || changeY || fabs(curX - endX) > 0.5)) {
      
      step(publisher, rate);
      bool flag = sideForAnalyze == LEFT ? left : right;
      if (wasStopY && !flag) {
        shift(publisher, rate);
        wasStopY = false;
        break;
      }
    }
    
    wasStopX = stop;
    rate.sleep();
    ros::spinOnce();
    float oldDirY = dirY;
    dirY = endY < curY ? 1 : -1;
    toY(publisher);
    changeY = false;
    while (!stop && (wasStopX || changeX|| fabs(curY - endY) > 0.5)) {
      
      step(publisher, rate);
      bool flag = sideForAnalyze == LEFT ? left : right;
      if (wasStopX && !flag) {
        shift(publisher, rate);
        wasStopX = false;
        break;
      }
    }
    wasStopY = stop;
    
  }

  return 0;
}