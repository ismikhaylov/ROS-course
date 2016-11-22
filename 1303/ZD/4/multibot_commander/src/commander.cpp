#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <iostream>

float _currentLeft;
float _currentRight;
float _currentFront;

float _minRange;
float _maxRange;
float _minRangeAngle;
float _maxRangeAngle;

ros::Time _maneuverStartTime;

tf::TransformListener * listener;

enum robotState{ ROBOT_LOST, FOUND_WALL, ALIGNMENT_START, ALIGNMENT, ATTACHED_TO_WALL, CORNER, GOAL_VISIBLE };
robotState _state;

bool isGoalVisible() {
    tf::StampedTransform transform;
    try {
        std::string err;
        ros::Time commonTime = ros::Time(0);
        //listener->waitForTransform("/target","/base_link",commonTime, ros::Duration(1));
        listener->getLatestCommonTime("/target","/base_link", commonTime, &err);
        listener->lookupTransform("/target","/base_link", commonTime, transform);
    } catch(tf::TransformException & ex) {
        ROS_INFO("E: %s",ex.what());
        ros::Duration(1).sleep();
        return false;
    }
    tf::Vector3 toTarget = transform.getOrigin();
    return (toTarget.length() < 1);
}

void scanRecv(const sensor_msgs::LaserScan::ConstPtr & msg) {
    _currentLeft = msg->ranges[msg->ranges.size() - 1];
    _currentFront = msg->ranges[msg->ranges.size() / 2];
    _currentRight = msg->ranges[0];
    _maxRange = msg->range_min;
    _minRange = msg->range_max;
    int minIndex = 0;
    int maxIndex = 0;
    for(int i=0; i<msg->ranges.size(); i++) {
        if(msg->ranges[i]>_maxRange){
            _maxRange = msg->ranges[i];
            maxIndex = i;
        }
        if(msg->ranges[i]<_minRange){
            _minRange = msg->ranges[i];
            minIndex = i;
        }
    }
    _minRangeAngle = msg->angle_min + static_cast<float>(minIndex) * msg->angle_increment;
    _maxRangeAngle = msg->angle_min + static_cast<float>(maxIndex) * msg->angle_increment;
}

void orientRobot(geometry_msgs::Twist & cmd) {
    if(_minRangeAngle > 0.03f){
        cmd.angular.z = 0.5;
    } else if(_minRangeAngle < -0.03f){
        cmd.angular.z = -0.5;
    } else {
        cmd.angular.z = 0;
        _state = FOUND_WALL;
    }
}

void moveToWall(geometry_msgs::Twist & cmd) {
    if(_currentFront > 0.45f){
        cmd.linear.x = 0.5;
    } else {
        cmd.linear.x = 0;
        _state = ALIGNMENT_START;
    }
}

void alignmentStart(geometry_msgs::Twist & cmd) {
    _maneuverStartTime = ros::Time::now();
    _state = ALIGNMENT;
}

void alignment(geometry_msgs::Twist & cmd) {
    if(_maneuverStartTime + ros::Duration(3) >= ros::Time::now()){
        cmd.angular.z = -0.53;
    } else {
        _state = ATTACHED_TO_WALL;
    }
}

void attachedMovement(geometry_msgs::Twist & cmd) {
    if(!isGoalVisible()){
            cmd.linear.x = 0.21;
            if(_currentFront <= 0.45) {
                cmd.linear.x = 0;
                cmd.angular.z = 0;
            }
            if(_currentLeft >= 0.51) {
                cmd.angular.z = 0.41;
            } else if (_currentLeft <= 0.49) {
                cmd.angular.z = -0.41;
            } else {
                if(_currentFront <= 0.45) {
                    _state = CORNER;
                    _maneuverStartTime = ros::Time::now();
                }
            }
    } else {
        _state = GOAL_VISIBLE;
    }
}

void cornerMovement(geometry_msgs::Twist & cmd) {
    if(_maneuverStartTime + ros::Duration(3) >= ros::Time::now()) {
        cmd.angular.z = -0.52;
    } else {
        _state = ATTACHED_TO_WALL;
    }
}

void resetRobot() {
    _state = ROBOT_LOST;
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "multibot_commander");
    ROS_INFO("Commander started!");
    resetRobot();
    ros::NodeHandle nh;
    ros::Subscriber scanSub = nh.subscribe("/base_scan", 1000, scanRecv);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    tf::TransformBroadcaster br;
    listener = new tf::TransformListener();
    ros::Rate loop_rate(100);
    sleep(1);
    tf::Vector3 targetPos(1,-3.4,0);
    tf::Transform targetTransform;
    targetTransform.setOrigin(targetPos);
    targetTransform.setRotation(tf::Quaternion(0,0,0,1));
    br.sendTransform(tf::StampedTransform(targetTransform,ros::Time::now(),"odom","target"));
    _maneuverStartTime = ros::Time::now();
    while(nh.ok()){
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        ros::spinOnce();
        switch(_state){
            case ROBOT_LOST:
                //std::cout << "LOST" << std::endl;
                orientRobot(cmd);
            break;
            case FOUND_WALL:
                //std::cout << "FOUND" << std::endl;
                moveToWall(cmd);
            break;
            case ALIGNMENT_START:
                //std::cout << "ALIGNMENT_START" << std::endl;
                alignmentStart(cmd);
            break;
            case ALIGNMENT:
                //std::cout << "ALIGNMENT" << std::endl;
                alignment(cmd);
            break;
            case ATTACHED_TO_WALL:
                //std::cout << "ATTACHED" << std::endl;
                attachedMovement(cmd);
            break;
            case CORNER:
                //std::cout << "CORNER" << std::endl;
                cornerMovement(cmd);
            break;
            case GOAL_VISIBLE:
                //std::cout << "GOAL" << std::endl;
            break;
        }
        pub.publish(cmd);
        br.sendTransform(tf::StampedTransform(targetTransform,ros::Time::now(),"odom","target"));
        loop_rate.sleep();
    }

    return 0;
}
