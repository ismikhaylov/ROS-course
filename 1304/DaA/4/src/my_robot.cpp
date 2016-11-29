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

enum RobotState {
    NOT_AVAILABLE = 0,
    LOOKING_FOR_WALL,
    LEFT_TURN,
    RIGHT_TURN,
    MOVE_ALONG,
    ADDITIONAL_STEP
};

class Robot {
private:
    RobotState movingState;
    bool isReached;
    ros::NodeHandle &nh;
    tf::TransformListener * listener;
    tf::Transform targetTransform;
    tf::TransformBroadcaster br;

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

        if (movingState == NOT_AVAILABLE) 
            movingState = LOOKING_FOR_WALL;
    }

    void updateDirection() {
        switch(movingState) {
            case NOT_AVAILABLE:
                break;
            case LOOKING_FOR_WALL:
                lookingForWall();
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
            default: break;
        }
    }

    void lookingForWall() {
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

    void leftTurn() {
        if (timeFoundWall + ros::Duration(1) >= ros::Time::now()) {
            av = M_PI / 2;
        } else {
            av = 0;
            lateRightFreeSpace = rightFreeSpace;
            movingState = MOVE_ALONG;
        }
    }

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

    void moveAlong() {
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

        isReached = verify();
    }

    void additionalStep() {
        if (frontFreeSpace > 4 * gap) {
            lv = frontFreeSpace / 2;
            av = 0;
        } else {
            movingState = RIGHT_TURN;
            timeFoundEndWall = ros::Time::now();
        }
    }

    bool verify() {
        try {
            tf::StampedTransform transform;
            std::string err;
            ros::Time commonTime = ros::Time(0);
            listener->getLatestCommonTime("/destination", "/base_link", commonTime, &err);
            listener->lookupTransform("/destination", "/base_link", commonTime, transform);
            tf::Vector3 toTarget = transform.getOrigin();
            return toTarget.length() < 1;
        } catch(tf::TransformException & ex) {
            ROS_INFO("Error while listen: %s", ex.what());
            ros::Duration(1).sleep();
            return false;
        }
    }

    void acceptReaching() {
        cout << "My robot has reached the destination point successfully";
    }

public:
    Robot(ros::NodeHandle &nh) : nh(nh) {
        movingState = NOT_AVAILABLE;

        lv = 0;
        av = 0;

        rightFreeSpace = 0;
        frontFreeSpace = 0;

        gap = 0.35;

        isReached = false;

        listener = new tf::TransformListener();
    }
    ~Robot(){}

    ros::Subscriber listen() {
        return nh.subscribe("/base_scan", 1000, &Robot::laserScanCallback, this);
    } 

    void start(ros::Publisher &pub) {
        geometry_msgs::Twist vel;

        ros::Rate rate(100);
        while(nh.ok()) {
            if (isReached) {
                acceptReaching();
                return;
            }
            updateDirection();

            vel.angular.z = av;
            vel.linear.x = lv;
            pub.publish(vel);
            ros::spinOnce();
            rate.sleep();
            br.sendTransform(tf::StampedTransform(targetTransform, ros::Time::now(), "odom", "destination"));
        }
    }

    void setup() {
        sleep(1);
        tf::Vector3 targetWayOut(0, -9, 0);
        targetTransform.setOrigin(targetWayOut);
        targetTransform.setRotation(tf::Quaternion(0,0,0,1));
        br.sendTransform(tf::StampedTransform(targetTransform, ros::Time::now(), "odom", "destination"));
    }

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "my_robot");
    ROS_INFO("My robot has started moving");
    sleep(5);

    ros::NodeHandle nh;
    Robot robot(nh);
    ros::Subscriber sub = robot.listen();
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    robot.setup();
    robot.start(pub);

    return 0;
}
