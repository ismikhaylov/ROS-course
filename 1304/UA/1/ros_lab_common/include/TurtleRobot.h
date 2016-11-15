#ifndef PROJECT_TURTLEROBOT_H
#define PROJECT_TURTLEROBOT_H

#include <rosconsole/macros_generated.h>
#include <ros/console_backend.h>

#include <boost/shared_ptr.hpp>

#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>

#include "Interpolator.h"
#include "GeometryUtils.h"

class TurtleRobot
{
public:
    TurtleRobot(ros::NodeHandle& node, const std::string& poseTopic)
            : m_angleInterpolator((const float) (M_PI / 30.f), (const float) (12 * M_PI / 15.f), 0, (const float) M_PI)
            , m_linearInterpolator(0.1f, 4.f, 0.f, 10.f)
    {
        chatter_pub = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
        subscriber = node.subscribe<turtlesim::Pose>(
                poseTopic,
                100,
                &TurtleRobot::setCurrentPosition,
                this);
    }

    float distanceTo(const turtlesim::Pose& goalPosition) const
    {
        return GeometryUtils::distance(m_position, goalPosition);
    }

    float getThetaDiff(const turtlesim::Pose& goalPosition) const
    {
        float angle;

        if (std::abs(goalPosition.y - m_position.y) < GeometryUtils::epsilon)
        {
            if (std::abs(goalPosition.x - m_position.x) < GeometryUtils::epsilon)
            {
                angle = 0.0;
            }
            else
            {
                angle = (float) ((goalPosition.x < m_position.x) ? M_PI : 0.0);
            }
        }
        else if (std::abs(goalPosition.x - m_position.x) < GeometryUtils::epsilon)
        {
            angle = (float) ((goalPosition.y < m_position.y) ? -M_PI / 2.f : M_PI / 2.f);
        }
        else
        {
            float tg = (goalPosition.y - m_position.y) / (goalPosition.x - m_position.x);
            angle = std::atan(tg);
            if (goalPosition.x < m_position.x)
            {
                angle += M_PI;
            }
        }

        return angle;
    }

    void moveToPoint(const turtlesim::Pose& goalPosition)
    {
        geometry_msgs::Twist twist;

        float thetaDiff = GeometryUtils::normAngle(getThetaDiff(goalPosition)) - GeometryUtils::normAngle(m_position.theta);
        thetaDiff = GeometryUtils::normAngle(thetaDiff);
        ROS_INFO("Theta delta: %f", thetaDiff * 180.f / M_PI);
        twist.linear.x = m_linearInterpolator.interpolate(distanceTo(goalPosition));
        twist.angular.z = 0.005f + m_angleInterpolator.interpolate(std::abs(thetaDiff));
        twist.angular.z = (thetaDiff > 0)? twist.angular.z: -twist.angular.z;
        ROS_INFO("New command position: x:%f y:%f theta:%f", twist.linear.x, twist.linear.y, twist.angular.z);

        chatter_pub.publish(twist);
    }

    void stopMovement()
    {
        chatter_pub.publish(geometry_msgs::Twist());
    }

    void setCurrentPosition(const boost::shared_ptr<const turtlesim::Pose>& newPosition)
    {
        m_position = *newPosition;
        m_newNewPositionKnown = true;
    }

    const turtlesim::Pose& getPosition() const
    {
        return m_position;
    }

    bool haveNewPosition() const
    {
        return m_newNewPositionKnown;
    }

    void resetNewPositionFlag()
    {
        m_newNewPositionKnown = false;
    }

private:
    ros::Publisher chatter_pub;
    ros::Subscriber subscriber;

    turtlesim::Pose m_position;
    bool m_newNewPositionKnown = false;
    Interpolator m_angleInterpolator;
    Interpolator m_linearInterpolator;
};

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <Interpolator.h>
#include "std_msgs/String.h"

#endif //PROJECT_TURTLEROBOT_H
