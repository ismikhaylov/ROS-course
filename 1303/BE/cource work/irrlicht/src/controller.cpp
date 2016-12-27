#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

/* LET'S USE PROCEDURE ORIENTED STYLE TO SIMPLIFY OUR LIFE */

using namespace std;
using namespace ros;
using namespace tf;
using namespace nav_msgs; 
using namespace geometry_msgs;

/* ACCESS TO ROS ENVIRONMENT */

NodeHandle * nodeHandle = 0;
TransformBroadcaster * tf_broadcaster = 0;
Publisher * odom_publisher = 0;
Publisher * irr_hero_pose_publisher = 0;

/* TIME FRAME HANDLING */

Time current_time;
Time last_time;

/* OUR CURRENT LOCATION AND VELOCITY */

double x = 1;
double y = 1;
double th = 0.0;

double vx = 0;
double vy = 0;
double vth = 0;

/* ALL CALLBACKS ARE DEFINED BELOW: */

void cmdCallback(const Twist & msg)
{
  vx = msg.linear.x;
  vy = msg.linear.y;
  vth = msg.angular.z;
}

/* ALL WORK FUNCTIONS AND PROCS ARE DEFINED BELOW: */

void updateLocation()
{
  // update time frame and velocity
  spinOnce();
  current_time = Time::now();

  // compute odometry in a typical way given the velocities of the robot
  double dt = (current_time - last_time).toSec();
  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  double delta_th = vth * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;
}

void publishOdometryOverTF()
{
  TransformStamped odometry_transform;
  odometry_transform.header.stamp = current_time;
  odometry_transform.header.frame_id = "odom";
  odometry_transform.child_frame_id = "base_link";

  odometry_transform.transform.translation.x = x;
  odometry_transform.transform.translation.y = y;
  odometry_transform.transform.translation.z = 0.0;
  odometry_transform.transform.rotation = createQuaternionMsgFromYaw(th);

  tf_broadcaster->sendTransform(odometry_transform);
}

void publishOdometryOverROS()
{
  Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = createQuaternionMsgFromYaw(th);;
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  odom_publisher->publish(odom);
}

void publishOdometry()
{
  publishOdometryOverTF();
  publishOdometryOverROS();

  last_time = current_time;
}

void initAll()
{
  // init ROS environment

  static NodeHandle n;
  nodeHandle = & n;

  static Publisher op = n.advertise<nav_msgs::Odometry>("odom", 500);
  odom_publisher = & op;

  static Subscriber sub = n.subscribe("/cmd_vel", 50, cmdCallback);

  static TransformBroadcaster odb;
  tf_broadcaster = & odb;

  static Publisher ihpp = n.advertise<PoseStamped>("/irrlicht/hero_pose", 500);
  irr_hero_pose_publisher = & ihpp;

  // init time frames
  current_time = Time::now();
  last_time = Time::now();
}

void publishAllToIrrlicht()
{
  PoseStamped msg;
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.orientation.z = th;
  irr_hero_pose_publisher->publish(msg);
}

int main(int argc, char ** argv)
{
  init(argc, argv, "controller");
  initAll();

  Rate r(1);

  while (nodeHandle->ok())
  {
    // listen to navigation stack and make an update based on a Twist
    updateLocation();

    // we have moved to somewhere, let's tell to navigation stack about our new location 
    publishOdometry();
    publishAllToIrrlicht();

    r.sleep();
  }
}