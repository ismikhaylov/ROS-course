#ifndef PROJECT_ROBOTCONTROLLER_H
#define PROJECT_ROBOTCONTROLLER_H


#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ModelState.h>

#include <thread>
#include <atomic>
#include <fstream>


using Point = std::pair<float, float>;


static const double EPSILON = 0.75;


class Util {
public:
    static float calculateDistance(const Point& from, const Point& to) {
        float dx = std::get<0>(to) - std::get<0>(from);
        float dy = std::get<1>(to) - std::get<1>(from);
        return calculateDistance(dx, dy);
    }

    static float calculateDistance(float dx, float dy) {
        return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    }

    static Point calculateVelocity(const Point& from, const Point& to) {
        float dx = std::get<0>(to) - std::get<0>(from);
        float dy = std::get<1>(to) - std::get<1>(from);
        return calculateVelocity(dx, dy);
    }

    static Point calculateVelocity(float dx, float dy) {
        float distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

        float linearSpeed = s_linearSpeed * distance;
        float angularSpeed = (float) (atan2(dy, dx));

        if (linearSpeed > s_maxSpeed) {
            linearSpeed = s_maxSpeed;
        }

        return {linearSpeed, angularSpeed};
    }

private:
    constexpr static const float s_maxSpeed = 0.05;
    constexpr static const float s_linearSpeed = 0.1;
};

class RobotController {
public:
    const std::string MODEL_PATH = "/home/alnen/.gazebo/models/pioneer2dx/model.sdf";

    RobotController(ros::NodeHandle& nodeHandle, const std::string& name, float x, float y)
            : m_name(name),
              m_position(x, y),
              m_angle(0.0f),
              m_broadcastingStooped(false) {
        m_markerPublisher = nodeHandle.advertise<visualization_msgs::Marker>("/" + name + "/marker", 10);
        m_gazeboPublisher = nodeHandle.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
        m_transformBroadcatster = std::thread([this] {
            auto broadcastRate = ros::Rate(60);
            while (!m_broadcastingStooped) {
                publishGlobalFrameTransform();
                broadcastRate.sleep();
            }
        });
        spawnRobot(nodeHandle, x, y);
    }

    bool getTransform(const std::string& target, tf::StampedTransform& transform) const {
        bool retval = false;

        try {
            auto currentTime = ros::Time::now();
            m_transformListener.waitForTransform("/" + m_name, "/" + target, currentTime, ros::Duration(1. / 60.));
            m_transformListener.lookupTransform("/" + m_name, "/" + target, currentTime, transform);
            retval = true;
            ROS_INFO("Successfully retrieved transformation %f, %f", transform.getOrigin().getX(),
                     transform.getOrigin().getY());
        }
        catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
        }

        return retval;
    }

    bool closeToTarget(const std::string& targetName) {
        bool retval = false;
        tf::StampedTransform transform;
        if (getTransform(targetName, transform)) {
            float dx = (float) transform.getOrigin().getX();
            float dy = (float) transform.getOrigin().getY();
            retval = Util::calculateDistance(dx, dy) < EPSILON;
        }
        return retval;
    }

    bool closeToTarget(const Point& targetPosition) {
        float dx = targetPosition.first - m_position.first;
        float dy = targetPosition.second - m_position.second;
        return Util::calculateDistance(dx, dy) < EPSILON;
    }

    void moveToTarget(const std::string& targetName) {
        tf::StampedTransform transform;
        if (getTransform(targetName, transform)) {
            float dx = (float) transform.getOrigin().getX();
            float dy = (float) transform.getOrigin().getY();
            Point targetPosition{getX() + dx, getY() + dy};
            moveToTarget(targetPosition);
        }
    }

    void moveToTarget(const Point& targetPosition) {
        float linearSpeed, angularSpeed;
        std::tie(linearSpeed, angularSpeed) = Util::calculateVelocity(m_position, targetPosition);
        getX() += linearSpeed * std::cos(angularSpeed);
        getY() += linearSpeed * std::sin(angularSpeed);
        m_angle = angularSpeed;
    }

    void publishGlobalFrameTransform() {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3{getX(), getY(), 0.0});
        transform.setRotation(tf::Quaternion{0.0, 0.0, 0.0, 1.0});

        m_transformBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/" + m_name));
        publishMarker();
        publishModelState();
    }

    const float getX() const { return std::get<0>(m_position); }

    const float getY() const { return std::get<1>(m_position); }

    float& getX() { return std::get<0>(m_position); }

    float& getY() { return std::get<1>(m_position); }

private:

    void publishMarker() {
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

    void publishModelState() {
        gazebo_msgs::ModelState robotState;
        robotState.model_name = m_name;
        robotState.pose.position.x = getX();
        robotState.pose.position.y = getY();
        robotState.pose.orientation.x = 0.0;
        robotState.pose.orientation.y = 0.0;
        robotState.pose.orientation.z = sin(m_angle / 2);
        robotState.pose.orientation.w = cos(m_angle / 2);
        m_gazeboPublisher.publish(robotState);
    }

    void spawnRobot(ros::NodeHandle& node, double x, double y, double z = 0.0) {
        ros::service::waitForService("gazebo/spawn_sdf_model");
        ros::ServiceClient add_robot = node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
        gazebo_msgs::SpawnModel srv;
        std::ifstream fin(MODEL_PATH.c_str());
        std::string model_xml;
        std::string buf;
        while (!fin.eof()) {
            getline(fin, buf);
            model_xml += buf + "\n";
        }
        srv.request.model_xml = model_xml;
        srv.request.model_name = m_name;
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        pose.orientation.w = 1.0;
        srv.request.initial_pose = pose;
        add_robot.call(srv);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    Point m_position;
    float m_angle;
    std::string m_name;

    ros::Publisher m_markerPublisher;
    ros::Publisher m_gazeboPublisher;
    tf::TransformListener m_transformListener;
    tf::TransformBroadcaster m_transformBroadcaster;

    std::thread m_transformBroadcatster;
    std::atomic_bool m_broadcastingStooped;
};

#endif //PROJECT_ROBOTCONTROLLER_H
