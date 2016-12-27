#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "tf/LinearMath/Transform.h"


float disLeft = 100;
float disRight = 100;
float disFront = 100;

float target_x = 7;
float target_y = 6;

float robot_x;
float robot_y;
float robot_angle = 0;

float speed = 1;

ros::Publisher pub;

#define PI 3.14159265


void updateWallInfo(const sensor_msgs::LaserScan& msg) 
{
  disRight = msg.ranges[0];
  disFront = msg.ranges[msg.ranges.size()/2 + 2];
  disLeft = msg.ranges[msg.ranges.size()-1];
}

void updateRobotPos(const nav_msgs::Odometry& msg) 
{
  tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                 msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  robot_angle = yaw;


  robot_x = msg.pose.pose.position.x;
  robot_y = msg.pose.pose.position.y; 
  }

void goForward()
{
  geometry_msgs::Twist vel_msg;     
  vel_msg.linear.x = speed;
  pub.publish(vel_msg);
}

void rotate(float angle)
{
  geometry_msgs::Twist vel_msg;     
  vel_msg.angular.z = angle;
  pub.publish(vel_msg);
}

bool isEnd()
{
  return (sqrt( pow(robot_x-target_x, 2) + pow(robot_y-target_y,2) ) < 1);
}

int main(int argc, char** argv) {
  if (argc < 3) {
    ROS_ERROR( "Target position not defined." );
    return - 1;
  }

  ros::init(argc, argv, "pathplanner");

  ros::NodeHandle n;
  pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Subscriber subscriber = n.subscribe("/base_scan", 1000, updateWallInfo);
  ros::Subscriber subscriber2 = n.subscribe("/base_pose_ground_truth", 1000, updateRobotPos);
  ros::Rate rate(5);

  int state = 0;
  float angle = 0;

  while (ros::ok() && !isEnd()) {

    switch (state)
    {
        case 0:
            ROS_INFO( "Rotate to target point" );
            angle = atan( (robot_y - target_y) / (robot_x - target_x) );
            rotate( 0.2 * PI );
            if (fabs( robot_angle - angle) < 0.1) {
                state = 1;
            }
        break;

        case 1:
            ROS_INFO( "Move to target point" );
            goForward();
            if (disFront <= 2 * speed) {
                state = 2;
            }
        break;

        case 2:
            ROS_INFO( "STATE 2." );
            if (disFront <=  2 * speed || disLeft == 0) {
                rotate( -0.2 * PI );
            } else {
                goForward();
                if (disLeft > 2 * speed) {
                    state = 0;
                }   
            }
        break;
    }

    ros::spinOnce();
    rate.sleep();
  } 
  return 0;
}