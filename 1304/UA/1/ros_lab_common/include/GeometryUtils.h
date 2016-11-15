#ifndef PROJECT_GEOMETRYUTILS_H
#define PROJECT_GEOMETRYUTILS_H

#include <ros/ros.h>
#include <turtlesim/Pose.h>

using Position = std::tuple<float, float>;

class GeometryUtils
{
public:
    static constexpr float epsilon = 0.01f;

    static float normAngle(float angle)
    {
        while (angle < -M_PI) angle += 2 * M_PI;
        while (angle > M_PI) angle -= 2 * M_PI;
        return angle;
    }

    static float distance(const turtlesim::Pose& pose1, const turtlesim::Pose& pose2)
    {
        return std::sqrt(
                std::pow((pose1.x - pose2.x), 2) +
                std::pow((pose1.y - pose2.y), 2));
    }
};

#endif //PROJECT_GEOMETRYUTILS_H
