

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <iostream>
#include <string>
#include <cmath>
#define _USE_MATH_DEFINES
using namespace std; 

enum MovingRobotState {
    INIT = 0,
    FIND_WALL,
    TURN_LEFT,
    TURN_RIGHT,
    MOVE_ALONG,
    ADD_STEP
};

class MovingRobot {
private:
    ros::NodeHandle &robot_node;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::TransformListener * listener;

    ros::Time timeStartWall;
    ros::Time timeEndWall;

    MovingRobotState movingState;

    bool isReachedEndPoint;

    float rightSpace;
    float frontSpace;
    float lateRightSpace;
    float linVel, angVel;
    float gap;
    void confirmationReachingEndPoint() {
        tf::StampedTransform tr;
        try {
            std::string err;
            ros::Time commonTime = ros::Time(0);
            listener->getLatestCommonTime("/endpoint", "/base_link", commonTime, &err);
            listener->lookupTransform("/endpoint", "/base_link", commonTime, tr);
        } catch(tf::TransformException & ex) {
            ROS_INFO("Error while listen: %s", ex.what());
            ros::Duration(1).sleep();
        }
        tf::Vector3 distEndPoint = tr.getOrigin();
        isReachedEndPoint = distEndPoint.length() < 1;
    }    
    void nextDir() {
        switch(movingState) {
            case INIT:
                break;
            case FIND_WALL:
                findWall();
                break;
            case TURN_LEFT:
                turnLeft();
                break;
            case MOVE_ALONG:
                moveAlong();
                break;
            case TURN_RIGHT:
                turnRight();
                break;
            case ADD_STEP:
                addStep();
                break;
            default:
                cout << "Uncked case!" << endl;
        }
    }

    void findWall() {
        if (frontSpace > gap) {
            linVel = frontSpace;
            angVel = 0;
        }
        else {
            linVel = 0;
            timeStartWall = ros::Time::now();
            movingState = TURN_LEFT;
        }
    }

    void turnLeft() {
        if (timeStartWall + ros::Duration(1) >= ros::Time::now()) {
            angVel = M_PI / 2;
        } else {
            angVel = 0;
            lateRightSpace = rightSpace;
            movingState = MOVE_ALONG;
        }
    }


    void turnRight() {
        if (timeEndWall + ros::Duration(1) >= ros::Time::now()) {
            linVel = 0;
            angVel = - M_PI / 2;
            lateRightSpace = rightSpace;
        } else {
            angVel = 0;
            linVel = 0;
            lateRightSpace = rightSpace;
            movingState = MOVE_ALONG;
        }
    }


    void moveAlong() {

        if (lateRightSpace < rightSpace / 3) {
            movingState = ADD_STEP;
            linVel = frontSpace / 2;
            return;
        } else {
            lateRightSpace = rightSpace;
        }

        if (frontSpace > gap) {
            linVel = frontSpace;
            angVel = 0;
        }
        else {
            linVel = 0;
            timeStartWall = ros::Time::now();
            movingState = TURN_LEFT;
        }

        confirmationReachingEndPoint();
    }


    void addStep() {
        if (frontSpace > 4 * gap) {
            linVel = frontSpace / 2;
            angVel = 0;
        } else {
            movingState = TURN_RIGHT;
            timeEndWall = ros::Time::now();
        }
    }

    void handleLaserScan(const sensor_msgs::LaserScan::ConstPtr & msg) {

        int laserData = msg->ranges.size();

        rightSpace = msg->ranges[0];
        frontSpace = msg->ranges[laserData / 2];

        if (movingState == INIT) movingState = FIND_WALL;

    }    


public:
    MovingRobot(ros::NodeHandle &robot_node) : robot_node(robot_node) {


        linVel = 0;
        angVel = 0;
        rightSpace = 0;
        frontSpace = 0;
        gap = 0.35;
        isReachedEndPoint = false;
        movingState = INIT;
        listener = new tf::TransformListener();
    }

    ros::Subscriber listenLaserScan() {
        return robot_node.subscribe("/base_scan", 1000, &MovingRobot::handleLaserScan, this);
    } 

    void findEndPoint(ros::Publisher &pub) {
        ros::Rate rate(100);
        geometry_msgs::Twist vel;
        while(robot_node.ok() && !isReachedEndPoint) {
            nextDir();
            vel.linear.x = linVel;
            vel.angular.z = angVel;
            pub.publish(vel);
            ros::spinOnce();
            rate.sleep();
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "endpoint"));
        }

        if (isReachedEndPoint) {
            cout << "The end point is reached!" << endl;
        }
    }

    void setEndPoint() {
        sleep(1);
        tf::Vector3 targetWayOut(0, -9, 0);
        transform.setOrigin(targetWayOut);
        transform.setRotation(tf::Quaternion(0,0,0,1));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "endpoint"));
    }

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "moving_robot");
    sleep(7);

    ros::NodeHandle robot_node;
    MovingRobot moving_robot(robot_node);
    ros::Subscriber sub = moving_robot.listenLaserScan();
    ros::Publisher pub = robot_node.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    moving_robot.setEndPoint();
    moving_robot.findEndPoint(pub);

    return 0;
}
