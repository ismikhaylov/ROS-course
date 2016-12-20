#include "Sense.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <random>
#include <chrono>
#include <iostream>
#include <limits>
#include "laser_scan/LaserScan.h"

Sense::Sense(const Field& field, geometry_msgs::Pose pos, double firingRange, const ros::NodeHandle &handler) : 
    field(field), activeTarget(nullptr), position(pos), firingRange(firingRange), nodeHandler(handler) {
        const int queueSize = 1;
        requestSubscriber = nodeHandler.subscribe("get_laser_scan", queueSize, &Sense::processMessage, this);
        bulletSubscriber = nodeHandler.subscribe("bullet_pos", queueSize, &Sense::checkPosition, this);
        publisher = nodeHandler.advertise<laser_scan::LaserScan>("/rrbot/laser/scan", queueSize);
        firePublisher = nodeHandler.advertise<std_msgs::String>("robot_fire", queueSize);
        idCounter = 0;
    }

void Sense::checkPosition(const geometry_msgs::Pose &msg) {
    if (activeTarget->onCoordinate(msg.position.y, msg.position.z)) {
         targets.erase(std::remove_if(targets.begin(), targets.end(), [&](const Target* target) {
            return target == activeTarget;
        }), targets.end());
        activeTarget->removeModel();
        Target *old = activeTarget;
        if (!targets.empty()) {
            Target* target = findNewTarget();

            target->activate(true);
            activeTarget->broadcastPosition();
            
        } else {
            end = true;
            activeTarget = nullptr;
        }
        delete old;
    } else {
        std::cout << "FAIL\n";
    }
   
}

void Sense::processMessage(const std_msgs::String &request) {
    if (request.data == "laser_scan") {
        if (idCounter % 2 == 1) {
            std_msgs::String msg;
            msg.data = "fire";
            firePublisher.publish(msg);
        }
        idCounter++;
        generateLaserScan(request, false);
    } else {
       generateLaserScan(request, true); 
    }
    
}

void Sense::generateLaserScan(const std_msgs::String &request, bool simple) {
    laser_scan::LaserScan msg;
    tf::StampedTransform transform;
    try {
        listener.lookupTransform("world", "/killer_pos", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1).sleep();

    }
    msg.y_increment = 0.1;
    msg.z_increment = 0.1;
    msg.y_min = transform.getOrigin().y() - 1;
    msg.z_min = 0;
    msg.y_max = msg.y_min + 2;
    msg.z_max = field.height;
    float z_max = field.height;
    if (simple) {
        z_max  = 0.05;
    }
    msg.ranges = std::vector<float>();
    msg.z_size = 0;
    for (float z = msg.z_min; z <= z_max; z+= msg.z_increment) {
        for (float y = msg.y_min; y <= msg.y_max; y+= msg.y_increment) {
            float value = std::numeric_limits<float>::infinity();
            for (Target *target : targets) {
                if (target->onCoordinate(y, z)) {
                    value = target->getPosition().position.x;
                }
            }
            float barrierValue = std::numeric_limits<float>::infinity();
            for (Barrier *barrier : barriers) {
                if (barrier->onCoordinate(y, z)) {
                    if (transform.getOrigin().x() < barrier->getPosition().position.x - barrier->getSizeX() / 2) {
                        barrierValue = barrier->getPosition().position.x - barrier->getSizeX() / 2;
                    } else if (transform.getOrigin().x() > barrier->getPosition().position.x - barrier->getSizeX() / 2 &&
                               transform.getOrigin().x() < barrier->getPosition().position.x + barrier->getSizeX() / 2) {
                        barrierValue = transform.getOrigin().x();
                    } else {
                        barrierValue = barrier->getPosition().position.x + barrier->getSizeX() / 2;
                    }
                    if (fabs(transform.getOrigin().x() - barrierValue) < fabs(transform.getOrigin().x() - value)) {
                        value = barrierValue;
                    }
                }
                
            }
 
            msg.ranges.push_back(value);
        }
        msg.z_size++;
    }

    msg.time = ros::Time::now().toSec();
    publisher.publish(msg);
}

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
        Speed speed = {0, (double)speedDistribution(generator) / 6000, 
                       (double)speedDistribution(generator) / 6000};


        pose.position.x = field.position.position.x + lengthDistribution(generator);
        pose.position.y = field.position.position.y + widthDistribution(generator);
        pose.position.z = field.position.position.z + heightDistribution(generator);

        Target *target = new Target(pose, nodeHandler, borders, speed);
        targets.push_back(target);
    }
}

void Sense::generateBarriers(unsigned int number) {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_int_distribution<unsigned long> sizeDistribution(0, 190);

    std::uniform_int_distribution<unsigned long> lengthDistribution(1, field.length - 1);
    std::uniform_int_distribution<unsigned long> widthDistribution(1, field.width - 1);

    for (int i = 0; i < number; i++) {
        geometry_msgs::Pose pose;
        double sizeZ = (double)sizeDistribution(generator) / 100 + 0.1;
        double sizeX = (double)sizeDistribution(generator) / 100 + 0.1;
        double sizeY = (double)sizeDistribution(generator) / 100 + 0.1;
        pose.position.x = field.position.position.x + lengthDistribution(generator);
        pose.position.y = field.position.position.y + widthDistribution(generator);
        pose.position.z = sizeZ / 2;

        Barrier *barrier = new Barrier(pose, nodeHandler,sizeX, sizeY, sizeZ);

        barriers.push_back(barrier);

        for (Target * target : targets) {
            if (target->getPosition().position.x <= pose.position.x + sizeX / 2 &&
                target->getPosition().position.x >= pose.position.x - sizeX / 2) {
                target->barriers.push_back(barrier);
            }
        }
    }
}

void Sense::work() {

    if (activeTarget == nullptr) {
        if (targets.empty()) {
            end = true;
            return;
        }
        Target* target = findNewTarget();

        target->activate(true);
        activeTarget->broadcastPosition();
    }
    
    for (int i = 0; i < 150; i++) {
        for (Target *target : targets) {
            target->makeStep();
        }
        ros::spinOnce();
        ros::Duration(0.2).sleep();
        if (activeTarget != nullptr) {
            activeTarget->broadcastPosition();
        }
        listener.waitForTransform("world", "/killer_pos", ros::Time(0), ros::Duration(1));
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
    if (fabs(positionSecond.position.x - positionFirst.position.x) <= firingRange) {
        return fabs(positionSecond.position.y - positionFirst.position.y);
    }
    
    return sqrt(pow(fabs(positionSecond.position.x - positionFirst.position.x) - firingRange, 2) +
                pow((positionSecond.position.y - positionFirst.position.y), 2));
}