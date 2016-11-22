#ifndef WANDERED_ROBOT_H
#define WANDERED_ROBOT_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

class WanderedRobot
{
public:
    enum class State { FreeMove, AttachToWall, MoveAlongWall, MoveAlongCorner, Succeed };
    enum class MoveStyle { ForwardMove, Rotate };

    WanderedRobot(ros::NodeHandle& node, double x, double y, double goalX, double goalY);
    void moveToGoal();

private:
    bool rotate(geometry_msgs::Twist& velocityMessage, double angle);
    bool forwardMove(geometry_msgs::Twist& velocityMessage, double distance, double precision);
    bool forwardHighPrecisionMove(geometry_msgs::Twist& velocityMessage, double distance, double precision);

    bool inFixedPoint();
    bool isUnattached();
    void updateGoalLine();
    int getDirection();
    bool isCrossGoalLine();
    bool isReachGoal();

    double normalizeAngle(double angle);
    double getAngleByDelta(double dx, double dy);
    double getAngleBySinCos(double angleSin, double angleCos);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& message);
    void poseCallback(const nav_msgs::Odometry::ConstPtr& message);

private:
    ros::Publisher m_velocityPublisher;
    ros::Subscriber m_scanSubscriber;
    ros::Subscriber m_poseSubscriber;

    double m_x;
    double m_y;
    double m_angle;

    double m_goalX;
    double m_goalY;
    double m_goalPrecision;

    State m_state;
    MoveStyle m_moveStyle;

    double m_minAngularSpeed;
    double m_maxAngularSpeed;
    double m_angularPrecision;

    double m_minLinearSpeed;
    double m_maxLinearSpeed;
    double m_linearPrecision;

    double m_stepAlongWall;
    double m_minBarrierRange;
    double m_currentBarrierAngle;
    double m_currentBarrierRange;
    double m_barrierRangePrecision;

    double m_k; // for equation of line to goal
    double m_b;
    double m_fixedX;
    double m_fixedY;
    double m_fixedDistanceToGoal;
    double m_epsilon;

    bool m_canMoveToGoal;
    bool m_needLeftRotation;
    bool m_needCornerRotation;
};

#endif // WANDERED_ROBOT_H
