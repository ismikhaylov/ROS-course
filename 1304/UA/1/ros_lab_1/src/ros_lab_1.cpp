#include <ros/ros.h>
#include <TurtleRobot.h>

class Controller
{
public:

    Controller(ros::NodeHandle& node, const std::vector<Position>& path)
        : m_robot(node, "/turtle1/pose")
    {
        m_path.resize(path.size());
        for (size_t i = 0; i < path.size(); ++i)
        {
            m_path[i].x = std::get<0>(path[i]);
            m_path[i].y = std::get<1>(path[i]);
        }
    }

    void setNewStep()
    {
        m_step = (m_step + 1) % m_path.size();
    }

    const turtlesim::Pose& getGoal() const
    {
        return m_path[m_step];
    }

    bool closeToGoal() const
    {
        return (m_robot.distanceTo(getGoal()) < 10 * GeometryUtils::epsilon);
    }

    void tryAct()
    {
        if (m_robot.haveNewPosition())
        {
            m_robot.resetNewPositionFlag();

            if (closeToGoal())
            {
                setNewStep();
            }
            m_robot.moveToPoint(getGoal());
        }
    }

private:
    TurtleRobot m_robot;
    std::vector<turtlesim::Pose> m_path;
    size_t m_step = 0;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle node;
    ros::Rate loop_rate(10);

    std::vector<Position> path = {
            Position{ 8,  8},
            Position{ 8,  2},
            Position{ 2,  2},
            Position{ 2,  8},
            Position{ 8,  8},
            Position{ 5.544445f, 5.544445f},
            Position{ 2,  8},
    };

    Controller controller(node, path);
    while (ros::ok())
    {
        ROS_INFO("New tick");
        controller.tryAct();
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO("---------");
    }


    return 0;
}