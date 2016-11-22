#ifndef STUPID_BOT_H
#define STUPID_BOT_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt32.h>
#include <tf/transform_listener.h>

struct Position {
    float x;
    float y;
};

struct Color {
    float red;
    float green;
    float blue;
    float alpha;
};

class Robot {
    
    uint32_t shape;
    Color color;
    ros::Publisher robotPublisher;

protected:
    unsigned id;
    ros::NodeHandle nodeHandler;
    Position position;
    ros::Subscriber subscriber;
    ros::Publisher publisher;
    tf::TransformListener listener;
    bool saved;

    Robot(Position pos, uint32_t shape, Color color, const ros::NodeHandle &handler);
    void broadcastPosition();
public:
    void setPosition(Position &pos);
    Position getPosition();
    void run(const Position &pos);
    bool isSaved();
};

class SimpleRobot : public Robot {
    static uint32_t simpleRobotShape;
    static Color simpleRobotColor;
    
public:
    SimpleRobot(Position pos, const ros::NodeHandle &handler);
    void sendHelpMessage();
    void follow(const std_msgs::UInt32 &msg);
 
};

class RescuerRobot : public Robot {
    static uint32_t rescuerRobotShape;
    static Color rescuerRobotColor;
    Position entryPosition;
    
public:
    RescuerRobot(Position pos, const ros::NodeHandle &handler);
    void findLost(const std_msgs::UInt32 &msg);
    bool hasFound(Position pos);
    void returnBack();
};

#endif