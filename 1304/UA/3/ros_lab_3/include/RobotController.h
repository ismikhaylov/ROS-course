#ifndef PROJECT_ROBOTCONTROLLER_H
#define PROJECT_ROBOTCONTROLLER_H


#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <thread>
#include <atomic>


using Point = std::pair<float, float>;


static const double EPSILON = 0.125;


class Util
{
public:
    static float calculateDistance(const Point& from, const Point& to)
    {
        float dx = std::get<0>(to) - std::get<0>(from);
        float dy = std::get<1>(to) - std::get<1>(from);
        return calculateDistance(dx, dy);
    }

    static float calculateDistance(float dx, float dy)
    {
        return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    }

    static Point calculateVelocity(const Point& from, const Point& to)
    {
        float dx = std::get<0>(to) - std::get<0>(from);
        float dy = std::get<1>(to) - std::get<1>(from);
        return calculateVelocity(dx, dy);
    }

    static Point calculateVelocity(float dx, float dy)
    {
        float distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

        float linearSpeed = s_linearSpeed * distance;
        float angularSpeed = (float) (s_angularSpeed * atan2(dy, dx));

        if (linearSpeed > s_maxSpeed)
        {
            linearSpeed = s_maxSpeed;
        }

        return {linearSpeed, angularSpeed};
    }

private:
    constexpr static const float s_maxSpeed = 0.05;
    constexpr static const float s_angularSpeed = 1.2;
    constexpr static const float s_linearSpeed = 0.1;
};


class RobotController
{
public:
    RobotController(ros::NodeHandle& nodeHandle, const std::string& name, float x, float y)
            : m_name(name),
              m_position(x, y),
              m_broadcastingStooped(false)
    {
        m_markerPublisher = nodeHandle.advertise<visualization_msgs::Marker>("/" + name + "/marker", 10);
        m_transformBroadcatster = std::thread([this]{
            auto broadcastRate = ros::Rate(60);
            while (!m_broadcastingStooped)
            {
                publishGlobalFrameTransform();
                broadcastRate.sleep();
            }
        });
    }

    bool getTransform(const std::string& target, tf::StampedTransform& transform) const
    {
        bool retval = false;

        try
        {
            auto currentTime = ros::Time::now();
            m_transformListener.waitForTransform("/" + m_name, "/" + target, currentTime, ros::Duration(1./60.));
            m_transformListener.lookupTransform("/" + m_name, "/" + target, currentTime, transform);
            retval = true;
            ROS_INFO("Successfully retrieved transformation %f, %f", transform.getOrigin().getX(), transform.getOrigin().getY());
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
        }

        return retval;
    }

    bool closeToTarget(const std::string& targetName)
    {
        bool retval = false;
        tf::StampedTransform transform;
        if (getTransform(targetName, transform))
        {
            float dx = (float) transform.getOrigin().getX();
            float dy = (float) transform.getOrigin().getY();
            retval = Util::calculateDistance(dx, dy) < EPSILON;
        }
        return retval;
    }

    bool closeToTarget(const Point& targetPosition)
    {
        float dx = targetPosition.first - m_position.first;
        float dy = targetPosition.second - m_position.second;
        return Util::calculateDistance(dx, dy) < EPSILON;
    }

    void moveToTarget(const std::string& targetName)
    {
        tf::StampedTransform transform;
        if (getTransform(targetName, transform))
        {
            float dx = (float) transform.getOrigin().getX();
            float dy = (float) transform.getOrigin().getY();
            Point targetPosition{getX() + dx, getY() + dy};
            moveToTarget(targetPosition);
        }
    }

    void moveToTarget(const Point& targetPosition)
    {
        float linearSpeed, angularSpeed;
        std::tie(linearSpeed, angularSpeed) = Util::calculateVelocity(m_position, targetPosition);
        getX() += linearSpeed * std::cos(angularSpeed);
        getY() += linearSpeed * std::sin(angularSpeed);
    }

    void publishGlobalFrameTransform()
    {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3{getX(), getY(), 0.0});
        transform.setRotation(tf::Quaternion{0.0, 0.0, 0.0, 1.0});

        m_transformBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/" + m_name));
        publishMarker();
    }

    const float getX() const { return std::get<0>(m_position); }
    const float getY() const { return std::get<1>(m_position); }

    float& getX() { return std::get<0>(m_position); }
    float& getY() { return std::get<1>(m_position); }

private:
    void publishMarker()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/" + m_name;
        marker.header.stamp = ros::Time::now();
        marker.ns = m_name;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.orientation.w = 1.0;

        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        marker.points.emplace_back();
        m_markerPublisher.publish(marker);
    }

    Point m_position;
    std::string m_name;

    ros::Publisher m_markerPublisher;
    tf::TransformListener m_transformListener;
    tf::TransformBroadcaster m_transformBroadcaster;

    std::thread m_transformBroadcatster;
    std::atomic_bool m_broadcastingStooped;
};

#endif //PROJECT_ROBOTCONTROLLER_H
