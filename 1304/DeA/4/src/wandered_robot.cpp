#include "wandered_robot.h"
#include <geometry_msgs/Twist.h>

WanderedRobot::WanderedRobot(ros::NodeHandle& node, double x, double y, double goalX, double goalY)
    : m_x(x), m_y(y), m_angle(0.0), m_goalX(goalX), m_goalY(goalY), m_goalPrecision(0.5),
      m_state(State::FreeMove), m_moveStyle(MoveStyle::ForwardMove),
      m_minLinearSpeed(0.2), m_maxLinearSpeed(1.0), m_linearPrecision(0.1),
      m_minAngularSpeed(0.5 * M_PI / 180), m_maxAngularSpeed(25.0 * M_PI / 180), m_angularPrecision(0.5 * M_PI / 180),
      m_stepAlongWall(0.2), m_minBarrierRange(0.4), m_currentBarrierAngle(0.0),
      m_currentBarrierRange(1000.0), m_barrierRangePrecision(0.2), // minRange = 0.6, precision = 0.3 (for test3)
      m_canMoveToGoal(true),
      m_needLeftRotation(false),
      m_needCornerRotation(false)
{
    m_velocityPublisher = node.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 1000);
    m_scanSubscriber = node.subscribe("/robot_0/base_scan", 100, &WanderedRobot::scanCallback, this);
    m_poseSubscriber = node.subscribe("/robot_0/base_pose_ground_truth", 100, &WanderedRobot::poseCallback, this);
}

void WanderedRobot::moveToGoal()
{
    geometry_msgs::Twist velocityMessage;
    velocityMessage.linear.x = 0.0;
    velocityMessage.linear.y = 0.0;
    velocityMessage.linear.z = 0.0;
    velocityMessage.angular.x = 0.0;
    velocityMessage.angular.y = 0.0;
    velocityMessage.angular.z = 0.0;

    // for rotate
    double distance = 0.0;
    double cornerX, cornerY;
    bool canRotate = false;
    bool needRotationToWall = false;
    m_canMoveToGoal = true;
    m_needLeftRotation = false;
    m_needCornerRotation = false;

    double dx, dy, rotateAngle;
    updateGoalLine();

    ros::Rate rate(30);
    while ((m_state != State::Succeed) && ros::ok())
    {
        ros::spinOnce();

        if (isReachGoal())
        {
           m_state = State::Succeed;
        }
        else if (m_canMoveToGoal)
        {
            ROS_INFO("m_canMoveToGoal = true, state: FreeMove");
            m_state = State::FreeMove;
            m_moveStyle = MoveStyle::Rotate;
            rotateAngle = normalizeAngle(getAngleByDelta(m_goalX - m_x, m_goalY - m_y));
            m_canMoveToGoal = false;
        }
        else if (m_needCornerRotation)
        {
            ROS_INFO("state: MoveAlongWall => MoveAlongCorner");
            m_state = State::MoveAlongCorner;
            m_moveStyle = MoveStyle::Rotate;
            rotateAngle = normalizeAngle(m_angle - M_PI / 4);
            distance = (m_minBarrierRange + m_barrierRangePrecision) * std::sqrt(2);
            cornerX = m_x + distance * std::cos(rotateAngle);
            cornerY = m_y + distance * std::sin(rotateAngle);
            m_needCornerRotation = false;
        }

        if (m_state == State::FreeMove)
        {
            if (m_moveStyle == MoveStyle::Rotate)
            {
                if (rotate(velocityMessage, normalizeAngle(rotateAngle - m_angle)))
                {
                    ROS_INFO("moveStyle: Rotate => ForwardMove");
                    m_moveStyle = MoveStyle::ForwardMove;
                }
            }
            else
            {
                if (m_currentBarrierRange <= m_minBarrierRange)
                {
                    ROS_INFO("state: FreeMove => AttachToWall");
                    needRotationToWall = true;
                    m_state = State::AttachToWall;
                    m_moveStyle = MoveStyle::ForwardMove;
                    rotateAngle = normalizeAngle(m_angle + m_currentBarrierAngle);
                    updateGoalLine();
                }
                else
                {
                    dx = m_goalX - m_x;
                    dy = m_goalY - m_y;
                    if (forwardMove(velocityMessage, std::sqrt(dx * dx + dy * dy), m_goalPrecision))
                    {
                        ROS_INFO("state: FreeMove => Succeed");
                        m_state = State::Succeed;
                    }
                }
            }
        }
        else if (m_state == State::AttachToWall)
        {
            if (m_moveStyle == MoveStyle::Rotate)
            {
                if (rotate(velocityMessage, normalizeAngle(rotateAngle - m_angle)))
                {
                    ROS_INFO("state: AttachToWall => MoveAlongWall");
                    m_state = State::MoveAlongWall;
                    m_moveStyle = MoveStyle::ForwardMove;
                }
            }
            else
            {
                if (needRotationToWall)
                {
                    needRotationToWall = !rotate(velocityMessage, normalizeAngle(rotateAngle - m_angle));
                }
                else
                {
                    canRotate = false;
                    if (m_currentBarrierRange > m_minBarrierRange + 0.05) // to wall
                    {
                        canRotate = forwardHighPrecisionMove(velocityMessage, m_currentBarrierRange - m_minBarrierRange, 0.05);
                    }
                    else if (m_currentBarrierRange < m_minBarrierRange - 0.05) // from wall
                    {
                        canRotate = forwardHighPrecisionMove(velocityMessage, m_currentBarrierRange - m_minBarrierRange, 0.05);
                    }
                    else
                    {
                        canRotate = forwardMove(velocityMessage, 0.0, m_barrierRangePrecision);
                    }
                    if (canRotate)
                    {
                        ROS_INFO("moveStyle: ForwardMove => Rotate");
                        rotateAngle = normalizeAngle(m_angle + M_PI / 2);
                        m_moveStyle = MoveStyle::Rotate;
                    }
                }
            }
        }
        else if (m_state == State::MoveAlongWall)
        {
            if (m_needLeftRotation)
            {
                ROS_INFO("MoveAlongWall: m_needLeftRotation = true");
                m_moveStyle = MoveStyle::Rotate;
                rotateAngle = normalizeAngle(m_angle + M_PI / 2);
                m_needLeftRotation = false;
            }
            if (m_moveStyle == MoveStyle::Rotate)
            {
                if (rotate(velocityMessage, normalizeAngle(rotateAngle - m_angle)))
                {
                    ROS_INFO("moveStyle: Rotate => ForwardMove");
                    m_moveStyle = MoveStyle::ForwardMove;
                }
            }
            else
            {
                dx = m_goalX - m_x;
                dy = m_goalY - m_y;
                if (!inFixedPoint() && isCrossGoalLine())
                {
                    ROS_INFO("cross goal line");
                    forwardMove(velocityMessage, 0.0, m_linearPrecision);
                    m_canMoveToGoal = true;
                }
                else if (isUnattached())
                {
                    ROS_INFO("state: MoveAlongWall => AttachToWall (is Untached)");
                    forwardMove(velocityMessage, 0.0, m_linearPrecision);
                    m_state = State::AttachToWall;
                    m_moveStyle = MoveStyle::ForwardMove;
                    needRotationToWall = true;
                    rotateAngle = normalizeAngle(m_angle - M_PI/2);
                }
                else
                {
                    forwardMove(velocityMessage, m_stepAlongWall, m_linearPrecision);
                }
            }
        }
        else if (m_state == State::MoveAlongCorner)
        {
            if (m_moveStyle == MoveStyle::Rotate)
            {
                if (rotate(velocityMessage, normalizeAngle(rotateAngle - m_angle)))
                {
                    ROS_INFO("moveStyle: Rotate => ForwardMove");
                    m_moveStyle = MoveStyle::ForwardMove;
                }
            }
            else
            {
                dx = cornerX - m_x;
                dy = cornerY - m_y;
                if (forwardMove(velocityMessage, std::sqrt(dx * dx + dy * dy), m_linearPrecision))
                {
                    ROS_INFO("state: MoveAlongCorner => AttachToWall");
                    m_state = State::AttachToWall;
                    m_moveStyle = MoveStyle::ForwardMove;
                    needRotationToWall = true;
                    rotateAngle = normalizeAngle(m_angle - M_PI / 4 - M_PI / 2);
                }
            }
        }
        rate.sleep();
    }
    ROS_INFO("state: Succeed");
}

bool WanderedRobot::rotate(geometry_msgs::Twist& velocityMessage, double angle)
{
    velocityMessage.linear.x = 0.0;
    if (std::abs(angle) < m_angularPrecision)
    {
        velocityMessage.angular.z = 0.0;
    }
    else
    {
        velocityMessage.angular.z = 0.5 * angle;
        if (std::abs(velocityMessage.angular.z) > m_maxAngularSpeed)
        {
            velocityMessage.angular.z = (angle < 0 ? -m_maxAngularSpeed : m_maxAngularSpeed);
        }
        else if (std::abs(velocityMessage.angular.z) < m_minAngularSpeed)
        {
            velocityMessage.angular.z = (angle < 0 ? -m_minAngularSpeed : m_minAngularSpeed);
        }
    }
    m_velocityPublisher.publish(velocityMessage);
    return (velocityMessage.angular.z == 0.0);
}

bool WanderedRobot::forwardMove(geometry_msgs::Twist& velocityMessage, double distance, double precision)
{
    velocityMessage.angular.z = 0.0;
    if (std::abs(distance) < precision)
    {
        velocityMessage.linear.x = 0.0;
    }
    else
    {
        velocityMessage.linear.x = 0.5 * distance;
        if (std::abs(velocityMessage.linear.x) > m_maxLinearSpeed)
        {
            velocityMessage.linear.x = (distance < 0 ? -m_maxLinearSpeed : m_maxLinearSpeed);
        }
        else if (std::abs(velocityMessage.linear.x) < m_minLinearSpeed)
        {
            velocityMessage.linear.x = (distance < 0 ? -m_minLinearSpeed : m_minLinearSpeed);
        }
    }
    m_velocityPublisher.publish(velocityMessage);
    return (velocityMessage.linear.x == 0.0);
}

bool WanderedRobot::forwardHighPrecisionMove(geometry_msgs::Twist& velocityMessage, double distance, double precision)
{
    velocityMessage.angular.z = 0.0;
    if (std::abs(distance) < precision)
    {
        velocityMessage.linear.x = 0.0;
    }
    else
    {
        velocityMessage.linear.x = 0.5 * distance;
    }
    m_velocityPublisher.publish(velocityMessage);
    return (velocityMessage.linear.x == 0.0);
}

bool WanderedRobot::inFixedPoint()
{
    return ((std::abs(m_x - m_fixedX) < 0.3) && (std::abs(m_y - m_fixedY) < 0.3));
}

bool WanderedRobot::isUnattached()
{
    return ((m_currentBarrierRange > m_minBarrierRange + m_barrierRangePrecision)
        || (m_currentBarrierRange < m_minBarrierRange - m_barrierRangePrecision));
}

void WanderedRobot::updateGoalLine()
{
    m_k = (m_goalY - m_y) / (m_goalX - m_x);
    m_b = m_y - m_k * m_x;
    m_fixedX = m_x;
    m_fixedY = m_y;
    double dx = m_goalX - m_x;
    double dy = m_goalY - m_y;
    m_fixedDistanceToGoal = std::sqrt(dx * dx + dy * dy);
}

int WanderedRobot::getDirection()
{
    m_epsilon = 0.1;
    if (std::abs(m_goalX - m_fixedX) < m_epsilon)
    {
        if (std::abs(m_goalX - m_x) > m_epsilon) return -1;
        if (((m_goalY - m_fixedY >= 0) && (m_goalY - m_y >= 0))
            || ((m_goalY - m_fixedY < 0) && (m_goalY - m_y < 0)))
        {
            return 1;
        }
        return 0;
    }
    else if (std::abs(m_goalY - m_fixedY) < m_epsilon)
    {
        if (std::abs(m_goalY - m_y) > m_epsilon) return -1;
        if (((m_goalX - m_fixedX >= 0) && (m_goalX - m_x >= 0))
            || ((m_goalX - m_fixedX < 0) && (m_goalX - m_x < 0)))
        {
            return 1;
        }
        return 0;
    }
    else
    {
        double y = m_k * m_x + m_b;
        if (std::abs(y - m_y) < m_epsilon)
        {
            if (((m_goalX - m_fixedX >= 0) && (m_goalX - m_x >= 0))
                || ((m_goalX - m_fixedX < 0) && (m_goalX - m_x < 0)))
            {
                return 1;
            }
            return 0;
        }
        return -1;
    }
    // -1 - cross, 0 - opposite, 1 - same
}

bool WanderedRobot::isCrossGoalLine()
{
    int direction = getDirection();
    double dx = m_goalX - m_x;
    double dy = m_goalY - m_y;
    double distance = std::sqrt(dx * dx + dy * dy);
    return !((direction < 0) || (distance > m_fixedDistanceToGoal));
}

bool WanderedRobot::isReachGoal()
{
    double dx = m_goalX - m_x;
    double dy = m_goalY - m_y;
    return (std::sqrt(dx * dx + dy * dy) < m_goalPrecision);
}

double WanderedRobot::normalizeAngle(double angle)
{
    while (angle < -M_PI) angle += 2 * M_PI;
    while (angle > M_PI) angle -= 2 * M_PI;
    return angle;
}

double WanderedRobot::getAngleByDelta(double dx, double dy)
{
    if (dy == 0) return (dx < 0 ? M_PI : 0.0);
    else
    {
        if (dx == 0) return (dy < 0 ? -M_PI/2 : M_PI/2);
        double angleCos = dx / std::sqrt(dx * dx + dy * dy);
        return (dy < 0 ? -std::acos(angleCos) : std::acos(angleCos));
    }
    return 0.0;
}

double WanderedRobot::getAngleBySinCos(double angleSin, double angleCos)
{
    if (angleSin == 0) return (angleCos < 0 ? M_PI : 0.0);
    else
    {
        if (angleCos == 0) return (angleSin < 0 ? -M_PI/2 : M_PI/2);
        return (angleSin < 0 ? -std::acos(angleCos) : std::acos(angleCos));
    }
    return 0.0;
}

void WanderedRobot::scanCallback(const sensor_msgs::LaserScan::ConstPtr& message)
{
    if (message->ranges.size() <= 0) return;
    double minRange = message->ranges[0];
    int minIndex = 0;
    double angle;
    for (int i = 0; i < message->ranges.size(); ++i)
    {
        if (message->ranges[i] < minRange)
        {
            minRange = message->ranges[i];
            minIndex = i;
        }
    }
    m_currentBarrierAngle = message->angle_min + (double)minIndex * message->angle_increment;
    m_currentBarrierRange = minRange;

    if ((m_state == State::MoveAlongWall) && (m_moveStyle == MoveStyle::ForwardMove))
    {
        if (message->ranges[message->ranges.size()/2] <= m_minBarrierRange)
        {
            m_needLeftRotation = true;
        }
        else
        {
            if (message->ranges[0] > 1.5 * m_minBarrierRange)
            {
                m_needCornerRotation = true;
            }
        }
    }
}

void WanderedRobot::poseCallback(const nav_msgs::Odometry::ConstPtr& message)
{
    m_x = message->pose.pose.position.x;
    m_y = message->pose.pose.position.y;
    m_angle = normalizeAngle(getAngleBySinCos(message->pose.pose.orientation.z, message->pose.pose.orientation.w) * 2);
}
