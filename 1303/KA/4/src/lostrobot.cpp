#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <iostream>
#include <string>
#include <cmath>

using namespace std; 

enum MovingState {
    INIT = 0,
    FIND_WALL,
    LEFT_TURN,
    RIGHT_TURN,
    MOVE_ALONG,
    ADDITIONAL_STEP
};

class LostRobot {
private:
    ros::NodeHandle &nh;
    tf::TransformListener * listener;
    tf::Transform targetTransform;
    tf::TransformBroadcaster br;

    MovingState movingState;
    bool isFoundWayOut;

    float rightFreeSpace;
    float frontFreeSpace;

    float lv, av;
    float gap;

    float lateRightFreeSpace;

    ros::Time timeFoundWall;
    ros::Time timeFoundEndWall;


    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr & msg) {
        int measurements = msg->ranges.size();

        rightFreeSpace = msg->ranges[0];
        frontFreeSpace = msg->ranges[measurements / 2];

        if (movingState == INIT) movingState = FIND_WALL;

    }

    void updateDirection() {
        switch(movingState) {
            case INIT:
                break;
            case FIND_WALL:
                findWall();
                break;
            case LEFT_TURN:
                leftTurn();
                break;
            case MOVE_ALONG:
                moveAlong();
                break;
            case RIGHT_TURN:
                rightTurn();
                break;
            case ADDITIONAL_STEP:
                additionalStep();
                break;
            default:
                cout << "Uncked case!" << endl;
        }
    }

    //find the wall.
    void findWall() {
        if (frontFreeSpace > gap) {
            lv = frontFreeSpace;
            av = 0;
        }
        else {
            lv = 0;
            timeFoundWall = ros::Time::now();
            movingState = LEFT_TURN;
        }
    }

    //use for right-hand rule, the turn to move along the wall.
    void leftTurn() {
        if (timeFoundWall + ros::Duration(1) >= ros::Time::now()) {
            av = M_PI / 2;
        } else {
            av = 0;
            lateRightFreeSpace = rightFreeSpace;
            movingState = MOVE_ALONG;
        }
    }

    //turn hold the wall.
    void rightTurn() {
        if (timeFoundEndWall + ros::Duration(1) >= ros::Time::now()) {
            lv = 0;
            av = - M_PI / 2;
            lateRightFreeSpace = rightFreeSpace;
        } else {
            av = 0;
            lv = 0;
            lateRightFreeSpace = rightFreeSpace;
            movingState = MOVE_ALONG;
        }
    }

    //move along the wall.
    void moveAlong() {
        //if found the end of wall.
        if (lateRightFreeSpace < rightFreeSpace / 3) {
            movingState = ADDITIONAL_STEP;
            lv = frontFreeSpace / 2;
            return;
        } else {
            lateRightFreeSpace = rightFreeSpace;
        }

        if (frontFreeSpace > gap) {
            lv = frontFreeSpace;
            av = 0;
        }
        else {
            lv = 0;
            timeFoundWall = ros::Time::now();
            movingState = LEFT_TURN;
        }

        verifyWayOut();
    }

    //some featch.
    void additionalStep() {
        if (frontFreeSpace > 4 * gap) {
            lv = frontFreeSpace / 2;
            av = 0;
        } else {
            movingState = RIGHT_TURN;
            timeFoundEndWall = ros::Time::now();
        }
    }

    //verify that a wayout is near.
    void verifyWayOut() {
        tf::StampedTransform transform;
        try {
            std::string err;
            ros::Time commonTime = ros::Time(0);
            listener->getLatestCommonTime("/wayOut", "/base_link", commonTime, &err);
            listener->lookupTransform("/wayOut", "/base_link", commonTime, transform);
        } catch(tf::TransformException & ex) {
            ROS_INFO("Error while listen: %s", ex.what());
            ros::Duration(1).sleep();
        }
        tf::Vector3 toTarget = transform.getOrigin();
        isFoundWayOut = toTarget.length() < 1;
    }

public:
    LostRobot(ros::NodeHandle &nh) : nh(nh) {
        movingState = INIT;

        lv = 0;
        av = 0;

        rightFreeSpace = 0;
        frontFreeSpace = 0;

        gap = 0.35;

        isFoundWayOut = false;

        listener = new tf::TransformListener();
    }
    ~LostRobot(){}

    ros::Subscriber listen() {
        return nh.subscribe("/base_scan", 1000, &LostRobot::laserScanCallback, this);
    } 

    void findWayOut(ros::Publisher &pub) {
        geometry_msgs::Twist vel;

        ros::Rate rate(100);
        while(nh.ok() && !isFoundWayOut) {
            updateDirection();

            vel.angular.z = av;
            vel.linear.x = lv;
            pub.publish(vel);
            ros::spinOnce();
            rate.sleep();
            br.sendTransform(tf::StampedTransform(targetTransform, ros::Time::now(), "odom", "wayOut"));
        }

        if (isFoundWayOut) {
            cout << "***** Way out is found! *****" << endl;
        }
    }

    void setWayOut() {
        sleep(1);
        tf::Vector3 targetWayOut(0, -9, 0);
        targetTransform.setOrigin(targetWayOut);
        targetTransform.setRotation(tf::Quaternion(0,0,0,1));
        br.sendTransform(tf::StampedTransform(targetTransform, ros::Time::now(), "odom", "wayOut"));
    }

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "lostrobot");
    ROS_INFO("The lost robot began to move!");
    sleep(5);

    ros::NodeHandle nh;
    LostRobot lostrobot(nh);
    ros::Subscriber sub = lostrobot.listen();
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    lostrobot.setWayOut();
    lostrobot.findWayOut(pub);

    return 0;
}
