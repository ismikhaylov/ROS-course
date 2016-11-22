#include "Robot.h"
#include <tf/transform_broadcaster.h>


Robot::Robot(Position pos, uint32_t shape, Color color, const ros::NodeHandle &handler) {
    nodeHandler = handler;
    robotPublisher = nodeHandler.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    this->shape = shape;
    this->color = color;
    setPosition(pos);
    broadcastPosition();
}

void Robot::setPosition(Position &pos) {
    position = pos;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/rf";
    marker.header.stamp = ros::Time::now();
    marker.ns = "robots";
    marker.id = id;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = position.x;
    marker.pose.position.y = position.y;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = color.red;
    marker.color.g = color.green;
    marker.color.b = color.blue;
    marker.color.a = color.alpha;

    marker.lifetime = ros::Duration();      // never auto-delete
    robotPublisher.publish(marker);
}

Position Robot::getPosition() {
    return position;
}

void Robot::broadcastPosition() {
    // Broadcast new position
    static tf::TransformBroadcaster broadcaster;
    std::string posTopic = "/robot_pos" + std::to_string(id);
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(position.x, position.y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", posTopic));
}

void Robot::run(const Position &pos) {
    ros::Rate rate(30);
    float newX = fabs(position.x - pos.x) / 100;
    newX *= position.x < pos.x ? 1 : -1;

    float newY = fabs(position.y - pos.y) / 100;
    newY *= position.y < pos.y ? 1 : -1;

    for (float step = 0; step < 100 && ros::ok(); step++) {
        Position newPos = {position.x + newX, position.y + newY};
        setPosition(newPos);
        broadcastPosition();
        rate.sleep();
    }   
} 


uint32_t SimpleRobot::simpleRobotShape = visualization_msgs::Marker::SPHERE;
Color SimpleRobot::simpleRobotColor = {0, 0, 1, 1};
SimpleRobot::SimpleRobot(Position pos, const ros::NodeHandle &handler) : 
    Robot(pos,simpleRobotShape,simpleRobotColor, handler) {
        id = rand() % 100;
        const int queueSize = 10;
        publisher = nodeHandler.advertise<std_msgs::UInt32>("help_msgs", queueSize);
        subscriber = nodeHandler.subscribe("instructions", 10, &SimpleRobot::follow, this);
        saved = false;
    }

void SimpleRobot::sendHelpMessage() {
    std_msgs::UInt32 msg;
    msg.data = id;
    publisher.publish(msg);
    ros::Rate rate(1);
    rate.sleep();
    ROS_INFO_STREAM("send");
    publisher.publish(msg);
}

void SimpleRobot::follow(const std_msgs::UInt32 &msg) {
    saved = true;
    unsigned rescureId = msg.data;
    ROS_INFO_STREAM(rescureId);
    std::string posTopic = "/robot_pos" + std::to_string(rescureId);
    listener.waitForTransform("/world", posTopic, ros::Time::now(), ros::Duration(1));

    ros::Rate rate(20);

    while (ros::ok()) {
        tf::StampedTransform transform;

        try {
            listener.lookupTransform("/world", posTopic, ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1).sleep();
            continue;
        }

        run({transform.getOrigin().x(), transform.getOrigin().y()});
        rate.sleep();
    }
}

bool Robot::isSaved() {
    return saved;
}

uint32_t RescuerRobot::rescuerRobotShape = visualization_msgs::Marker::CUBE;
Color RescuerRobot::rescuerRobotColor = {1, 0, 0, 1};
RescuerRobot::RescuerRobot(Position pos, const ros::NodeHandle &handler) : 
    Robot(pos,rescuerRobotShape,rescuerRobotColor, handler), entryPosition(pos) {
        id = rand() % 100 + 100; 
        subscriber = nodeHandler.subscribe("help_msgs" , 10, &RescuerRobot::findLost, this);
    }

void RescuerRobot::findLost(const std_msgs::UInt32 &msg) {
    unsigned lostId = msg.data;
    ROS_INFO_STREAM(lostId);
    publisher = nodeHandler.advertise<std_msgs::UInt32>("instructions", 10);
    std::string posTopic = "/robot_pos" + std::to_string(lostId);
    listener.waitForTransform("/world", posTopic, ros::Time::now(), ros::Duration(1));
    ros::Rate rate(10);
    while (ros::ok()) {
        tf::StampedTransform transform;

        try {
            listener.lookupTransform("/world", posTopic, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1).sleep();
            continue;
        }

        if (hasFound({transform.getOrigin().x(), transform.getOrigin().x()})) {

            std_msgs::UInt32 msg;
            ROS_INFO_STREAM("send");
            msg.data = id;

            publisher.publish(msg);
            ros::Duration(1).sleep();
            returnBack();
            return;
        }

        run({transform.getOrigin().x(), transform.getOrigin().y()});
        rate.sleep();
    }
}

bool RescuerRobot::hasFound(Position pos) {
    float x = fabs(position.x - pos.x);
    float y = fabs(position.y - pos.y);
    ROS_INFO_STREAM(x);
    ROS_INFO_STREAM(y);
    return (x < 1 && y < 0.4) || ( x < 0.4 && y < 1);
}

void RescuerRobot::returnBack() {
    run(entryPosition);
    ros::Rate rate(100);
}