#include <ros/ros.h>
#include "RobotController.h"
#include <random>


enum class State
{
    WAITING,
    FOLLOWING
};


class RandomPositionGenerator
{
public:
    static constexpr float MIN_VALUE = -2.f;
    static constexpr float MAX_VALUE =  2.f;

    RandomPositionGenerator(float originX, float originY, float minValue=MIN_VALUE, float maxValue=MAX_VALUE) :
            m_originX(originX),
            m_originY(originY),
            m_rd(MIN_VALUE, MAX_VALUE)
    {}

    Point getNewGoal()
    {
        return {m_originX + m_rd(m_re), m_originY + m_rd(m_re)};
    }

private:
    float m_originX;
    float m_originY;
    std::default_random_engine m_re;
    std::uniform_real_distribution<float> m_rd;
};


int main(int argc, char** argv) {
    std::string robotName = "soul";
    std::string targetName = "ashen_one";

    ros::init(argc, argv, robotName);
    ros::NodeHandle nodeHandle;

    RobotController robot(nodeHandle, robotName, -3.f, 3.f);
    State robotState = State::WAITING;

    RandomPositionGenerator rpg(robot.getX() + 6.f, robot.getY() - 6.f);
    Point randomPosition = rpg.getNewGoal();

    ros::Rate mainLoopRate(15);
    while (ros::ok())
    {
        if (robotState == State::WAITING)
        {
            if (!robot.closeToTarget(targetName))
            {
                if (robot.closeToTarget(randomPosition))
                {
                    randomPosition = rpg.getNewGoal();
                }
                robot.moveToTarget(randomPosition);
            }
            else
            {
                robotState = State::FOLLOWING;
            }
        }
        else
        {
            robot.moveToTarget(targetName);
        }

        ros::spinOnce();
        mainLoopRate.sleep();
    }

    return 0;
}