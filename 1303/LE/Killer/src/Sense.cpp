#include "Sense.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <random>
#include <chrono>
#include <iostream>
#include <limits>

Sense::Sense(const Field& field, geometry_msgs::Pose pos, double firingRange, const ros::NodeHandle &handler) : 
    field(field), activeTarget(nullptr), position(pos), firingRange(firingRange), nodeHandler(handler) {};

void Sense::generateTargets(unsigned int number) {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_int_distribution<unsigned char> speedDistribution(-100,100);

    std::uniform_int_distribution<unsigned long> lengthDistribution(0, field.length - 1);
    std::uniform_int_distribution<unsigned long> widthDistribution(0, field.width - 1);
    std::uniform_int_distribution<unsigned long> heightDistribution(1, field.height - 1);

    Borders borders = {field.position.position.y,
                       field.position.position.y + field.width,
                       field.position.position.z,
                       field.position.position.z + field.height};
    
    tf::Quaternion q(tf::Vector3(0, 1, 0), M_PI / 2);
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(q, odom_quat);
    for (int i = 0; i < number; i++) {
        geometry_msgs::Pose pose;
        pose.orientation = odom_quat;
        Speed speed = {(double)speedDistribution(generator) / 6000, 
                       (double)speedDistribution(generator) / 6000};

        pose.position.x = field.position.position.x + lengthDistribution(generator);
        pose.position.y = field.position.position.y + widthDistribution(generator);
        pose.position.z = field.position.position.z + heightDistribution(generator);

        Target *target = new Target(pose, nodeHandler, borders, speed);
        targets.push_back(target);
    }
}

void Sense::work() {
    if (activeTarget != nullptr) {
        activeTarget->activate(false);
    }
    Target* target = findNewTarget();
    
    if (target != nullptr) {
        target->activate(true);
        /*std::cout << "New target: " << target->getId() << "\n";
        std::cout << target->getPosition().position.x << ", " <<
                 target->getPosition().position.y << ", " <<
                 target->getPosition().position.z << "\n";*/
    }
    
    ros::Rate rate(5);
    for (int i = 0; i < 50; i++) {
        for (Target *target : targets) {
            target->makeStep();
        }
        rate.sleep();
    }
}

Target* Sense::findNewTarget() {
    double minDistance = std::numeric_limits<double>::max();
    if (activeTarget == nullptr) {
        for (Target *target : targets) {
            double distance = findDistance(position, target->getPosition());
            if (distance < minDistance) {
                minDistance = distance;
                activeTarget = target;
            }
        }
    } else {
        if (targets.size() <= 1) {
            return nullptr;
        }
        Target *oldActiveTarget = activeTarget;
        for (Target *target : targets) {
            if (target != oldActiveTarget) {
                double distance = findDistance(oldActiveTarget->getPosition(), target->getPosition());
                if (distance < minDistance) {
                    minDistance = distance;
                    activeTarget = target;
                }
            }
        }
    }
    return activeTarget;
}

double Sense::findDistance(const geometry_msgs::Pose& positionFirst,
                           const geometry_msgs::Pose& positionSecond) {
    if (abs(positionSecond.position.x - positionFirst.position.x) <= firingRange) {
        return abs(positionSecond.position.y - positionFirst.position.y);
    }
    return sqrt(pow(abs(positionSecond.position.x - positionFirst.position.x) - firingRange, 2) +
                pow((positionSecond.position.y - positionFirst.position.y), 2)/* +
                pow((positionSecond.position.z - positionFirst.position.z), 2)*/);
}