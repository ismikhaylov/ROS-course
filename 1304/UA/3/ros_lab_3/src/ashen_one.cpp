#include <ros/ros.h>
#include "RobotController.h"


class ExitHolder
{
public:
    ExitHolder(ros::NodeHandle& nodeHandle, float x, float y)
        : m_exitPosition{x, y}
    {
        m_exitMarkerPublisher = nodeHandle.advertise<visualization_msgs::Marker>("/goal/marker", 10);
    }

    void publishExitMarket() const
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "goal";
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.w = 1.0;

        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        geometry_msgs::Point point;
        point.x = std::get<0>(m_exitPosition);
        point.y = std::get<1>(m_exitPosition);
        marker.points.push_back(point);
        m_exitMarkerPublisher.publish(marker);
    }

    const float getX() const { return std::get<0>(m_exitPosition); }
    const float getY() const { return std::get<1>(m_exitPosition); }
    const Point& getPosition() const { return m_exitPosition; }

    float& getX() { return std::get<0>(m_exitPosition); }
    float& getY() { return std::get<1>(m_exitPosition); }
    Point& getPosition() { return m_exitPosition; }

private:
    Point m_exitPosition;
    ros::Publisher m_exitMarkerPublisher;
};

enum class State
{
    FOLLOWING,
    RETURNING_TO_BASE
};


int main(int argc, char** argv) {
    std::string robotName = "ashen_one";
    std::string targetName = "soul";

    ros::init(argc, argv, robotName);
    ros::NodeHandle nodeHandle;

    RobotController robot(nodeHandle, robotName, 3.f, 3.f);
    ExitHolder exitHolder(nodeHandle, -3.f, -3.f);

    State robotState = State::FOLLOWING;

    ros::Rate mainLoopRate(15);
    while (ros::ok())
    {
        robot.publishGlobalFrameTransform();
        if (robotState == State::FOLLOWING)
        {
            robot.moveToTarget(targetName);
            if (robot.closeToTarget(targetName))
            {
                robotState = State::RETURNING_TO_BASE;
            }
            robot.publishGlobalFrameTransform();
        }
        else
        {
            robot.moveToTarget(exitHolder.getPosition());
        }

        exitHolder.publishExitMarket();
        ros::spinOnce();
        mainLoopRate.sleep();
    }
    return 0;
}