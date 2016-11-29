#ifndef SENSE_H
#define SENSE_H

#include <ros/ros.h>
#include <vector>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include "Robot.h"

struct Field {
    unsigned int width;
    unsigned int length;
    unsigned int height;
    geometry_msgs::Pose position;
};

class Sense {
    Field field;
    Target *activeTarget;
    geometry_msgs::Pose position;
    double firingRange;
    ros::NodeHandle nodeHandler;
    std::vector<Target *> targets;

    double findDistance(const geometry_msgs::Pose& positionFirst,
                        const geometry_msgs::Pose& positionSecond);
public:
    Sense(const Field& field, geometry_msgs::Pose pos, double firingRange, const ros::NodeHandle &handler);
	void generateTargets(unsigned int number);
    void work();
    Target *findNewTarget();
};

#endif