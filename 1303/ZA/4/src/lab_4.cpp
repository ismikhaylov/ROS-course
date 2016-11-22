#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <algorithm>
#include <math.h>

#define PI 3.14159265

bool rotate_to_goal = false;
const float min_distance = 0.45;
int rotate_direction = 1;

float goal_x = -8;
float goal_y = 8;
const int min = 0;
const int max = 4;

float pos_x = 100;
float pos_y = 100;
float orientation_w = 0;

float center = 0;
float right = 0;
float left = 0;
float scan_min = 0;

int attempt = 0;

int count_step = 0;
int max_count_step = 100;

void callBackPoseRobot(const  nav_msgs::Odometry::ConstPtr& odometry) {
	pos_x = odometry->pose.pose.position.x;
	pos_y = odometry->pose.pose.position.y;
	if (odometry->pose.pose.orientation.w < 0.0 || odometry->pose.pose.orientation.z < 0.0) {
		if (odometry->pose.pose.orientation.w < 0.0){
			orientation_w = (-1) * asin(odometry->pose.pose.orientation.z) * 360 / PI;
		} else {
			orientation_w = (-1) * acos(odometry->pose.pose.orientation.w) * 360 / PI;
		}
	} else {
		orientation_w = acos(odometry->pose.pose.orientation.w) * 360 / PI;
	}
}

void callBackLaserInfo(const sensor_msgs::LaserScan::ConstPtr& scan) {
	center = scan->ranges[scan->ranges.size()/2];
	right = scan->ranges[0];
	left = scan->ranges[scan->ranges.size() - 1];
	scan_min = *std::min_element(scan->ranges.begin(), scan->ranges.end());	
	if (left > scan_min && right > scan_min) {
		center = scan_min;
	}
}
 
int  main (int argc, char **argv) {
	srand (time(NULL));
	ros::init(argc, argv, "lab_4");
	ros::NodeHandle n;
	ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Subscriber laser_subscriber = n.subscribe("/base_scan", 1000, callBackLaserInfo);
	ros::Subscriber pose_subscriber = n.subscribe("/base_pose_ground_truth", 1000, callBackPoseRobot);
	ros::Rate loop_rate(30);

	while (ros::ok()) {
		geometry_msgs::Twist vel_msg;
		if (right <= min_distance || left <= min_distance || center <= min_distance) {
			if (rotate_to_goal) {
				rotate_to_goal = false;
				count_step = (-1) * attempt * max_count_step;
			}
			vel_msg.linear.x = 0;
			vel_msg.angular.z = 0.9 * rotate_direction;
		} else { 
			vel_msg.linear.x = 0.5;
			vel_msg.angular.z = 0.0;
			if (right > left) {
				rotate_direction = -1;
			} else {
				rotate_direction = 1;
			}
		}
		
		if (count_step == max_count_step) {
			float angle = atan2(goal_y - pos_y, goal_x - pos_x) * 180/PI;
			if(fabs(orientation_w - angle) > 7) {
   				vel_msg.angular.z = 0.6;
				vel_msg.linear.x = 0;
			} else {
				rotate_to_goal = true;
				count_step = 0;	
				if (center < 0.7) {
					attempt = min + rand() % (max - min);
				} else {
					attempt = 0;
				}
			}	
		} else {
			count_step++;
		}

		if (fabs(goal_x - pos_x) < 0.2 && fabs(goal_y - pos_y) < 0.2) {
			break;
		}

		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
}
