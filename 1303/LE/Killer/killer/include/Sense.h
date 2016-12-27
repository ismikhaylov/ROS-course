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
    std::vector<Barrier *> barriers;
    ros::Subscriber requestSubscriber;
    ros::Subscriber bulletSubscriber;
    ros::Publisher publisher;
    ros::Publisher firePublisher;
    tf::TransformListener listener;

    double findDistance(const geometry_msgs::Pose& positionFirst,
                        const geometry_msgs::Pose& positionSecond);

    unsigned int idCounter;
public:
    bool end;
    Sense(const Field& field, geometry_msgs::Pose pos, double firingRange, const ros::NodeHandle &handler);
	void generateTargets(unsigned int number);
    void generateBarriers(unsigned int number);
    void work();
    Target *findNewTarget();
    void generateLaserScan(const std_msgs::String &request, bool simple);
    void checkPosition(const geometry_msgs::Pose &msg);
    void processMessage(const std_msgs::String &request);
    
};

#endif