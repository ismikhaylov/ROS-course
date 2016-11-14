#ifndef ROBOT_INFO_H
#define ROBOT_INFO_H

#include <string>

class RobotInfo
{
public:
    RobotInfo(const std::string& name);
    RobotInfo(const std::string& name, double distancePrecision, double angularPrecision, double currentAngle,
              double linearSpeed, double angularSpeed, double maxLinearSpeed);
    void setPosition(double x, double y);
    double getDistancePrecision();
    double getAngularPrecision();
    double getCurrentAngle();
    double getLinearSpeed();
    double getAngularSpeed();
    double getMaxLinearSpeed();
    double getX();
    double getY();
    std::string getName();
    void updatePosition(double dx, double dy, double distance);

private:
    double getRotateAngle(double dx, double dy);

private:
    double mDistancePrecision;
    double mAngularPrecision;
    double mCurrentAngle;
    double mLinearSpeed;
    double mAngularSpeed;
    double mMaxLinearSpeed;
    double mX;
    double mY;
    std::string mName;
};

#endif // ROBOT_INFO_H
