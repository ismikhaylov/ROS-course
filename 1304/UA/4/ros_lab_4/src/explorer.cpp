#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

using Pose2D = geometry_msgs::Pose2D;

const float ANGLE_EPSILON = 0.05;
const float ANGULAR_SPEED = 0.4f;
const float LINEAR_SPEED = 1.0f;
const float DISTANCE_EPSILON = 1.0f;
const unsigned int FRAME_LIMIT = 20;

class Parameters {

public:
    Parameters(int argc, char** argv) {
        if (argc == 4) {
            m_targetPosition.x = (float) atof(argv[1]);
            m_targetPosition.y = (float) atof(argv[2]);
            m_targetPosition.theta = (float) atof(argv[3]);
            ROS_INFO("Target position x: %f y: %f theta: %f",
                     m_targetPosition.x,
                     m_targetPosition.y,
                     m_targetPosition.theta);
        } else {
            ROS_ERROR("Must have 3 parameters. Actual count(%d):", argc);
            for (int i = 0; i < argc; ++i) {
                ROS_ERROR(" - %s", argv[i]);
            }
            throw std::runtime_error("Must have 3 parameters");
        }
    }

    const Pose2D& getTargetPosition() const {
        return m_targetPosition;
    }

private:
    Pose2D m_targetPosition;
};

class ExplorationHelper {
public:

    enum class State {
        FORWARD, BACK, LEFT, RIGHT, FINISH
    };

    ExplorationHelper(const Parameters& parameters, ros::NodeHandle& nodeHandle):
            m_targetPosition(parameters.getTargetPosition()) {
        m_odometrySubscriber = nodeHandle.subscribe<nav_msgs::Odometry>("/odom", 100, &ExplorationHelper::updatePosition, this);
        m_laserScanSubsriber = nodeHandle.subscribe<sensor_msgs::LaserScan>("/base_scan", 100, &ExplorationHelper::anaizeLaserScans, this);
        m_cmdVelPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    }

    const Pose2D& getCurrentPosition() const {
        return m_currentPosition;
    }

    const Pose2D& getTargetPosition() const {
        return m_targetPosition;
    }

    float getFrontRange() const {
        return m_frontRange;
    }

    float getLeftRange() const {
        return m_leftRange;
    }

    float getRightRange() const {
        return m_rightRange;
    }

    State getState() const {
        return m_state;
    }

    void setState(State state) {
        m_state = state;
        m_frame = 0;
    }

    unsigned int getFrame() const {
        return m_frame;
    }

    void incrementFrame() {
        ++m_frame;
    }

    float getAngleToTarget() const {
        float dx = (float) (m_targetPosition.x - m_currentPosition.x);
        float dy = (float) (m_targetPosition.y - m_currentPosition.y);
        return (float) (atan2(dy, dx) - m_currentPosition.theta);
    }

    float getDistanceToTarget() const {
        return (float) sqrt(
                        pow(m_currentPosition.x - m_targetPosition.x, 2.0) +
                        pow(m_currentPosition.y - m_targetPosition.y, 2.0));
    }

    void moveForward(float linearSpeed) {
        geometry_msgs::Twist message;
        message.linear.x = linearSpeed;
        m_cmdVelPublisher.publish(message);
    }

    void rotate(float angularSpeed) {
        geometry_msgs::Twist message;
        message.angular.z = angularSpeed;
        m_cmdVelPublisher.publish(message);
    }

    void stop() {
        m_cmdVelPublisher.publish(geometry_msgs::Twist());
    }

private:
    void updatePosition(nav_msgs::Odometry odometry) {
        m_currentPosition.x = odometry.pose.pose.position.x;
        m_currentPosition.y = odometry.pose.pose.position.y;
        m_currentPosition.theta = tf::getYaw(odometry.pose.pose.orientation);
    }

    void anaizeLaserScans(sensor_msgs::LaserScan laserScan) {
        unsigned long middle = laserScan.ranges.size() / 2;
        m_leftRange = laserScan.ranges[middle * 5 / 3];
        m_frontRange = laserScan.ranges[middle];
        m_rightRange = laserScan.ranges[middle / 3];
    }

    ros::Subscriber m_odometrySubscriber;
    ros::Subscriber m_laserScanSubsriber;
    ros::Publisher m_cmdVelPublisher;

    Pose2D m_currentPosition;
    Pose2D m_targetPosition;
    float m_frontRange = std::numeric_limits<float>::lowest();
    float m_leftRange = std::numeric_limits<float>::lowest();
    float m_rightRange = std::numeric_limits<float>::lowest();
    State m_state = State::FORWARD;
    unsigned int m_frame = 0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "explorer");

    Parameters parameters(argc, argv);

    ros::NodeHandle nodeHandle;
    ExplorationHelper explorer(parameters, nodeHandle);

    ros::Rate r(60);
    const Pose2D& currentPosition = explorer.getCurrentPosition();
    while (ros::ok()) {
        ROS_INFO("Current state %d", explorer.getState());
        ROS_INFO("Current position{x: %f y: %f theta: %f}",
                 currentPosition.x,
                 currentPosition.y,
                 currentPosition.theta);
        ROS_INFO("Current ranges{left: %f front: %f right: %f}",
                 explorer.getLeftRange(),
                 explorer.getFrontRange(),
                 explorer.getRightRange());
        ROS_INFO("Distance to target %f", explorer.getDistanceToTarget());

        float angleToTarget = explorer.getAngleToTarget();

        switch (explorer.getState()) {
            case ExplorationHelper::State::FORWARD:
            {
                if (explorer.getDistanceToTarget() < 0.1) {
                    explorer.setState(ExplorationHelper::State::FINISH);
                }
                if (fabs(angleToTarget) > ANGLE_EPSILON) {
                    if (angleToTarget < 0.0) {
                        explorer.rotate(-ANGULAR_SPEED);
                    } else {
                        explorer.rotate(ANGULAR_SPEED);
                    }
                    ROS_INFO("Angle %f", angleToTarget);
                } else {
                    if (explorer.getFrontRange() > DISTANCE_EPSILON) {
                        explorer.moveForward(LINEAR_SPEED);
                    } else if (explorer.getLeftRange() > DISTANCE_EPSILON) {
                        explorer.setState(ExplorationHelper::State::LEFT);
                    } else if (explorer.getRightRange() > DISTANCE_EPSILON) {
                        explorer.setState(ExplorationHelper::State::RIGHT);
                    } else {
                        explorer.setState(ExplorationHelper::State::BACK);
                    }
                }
                break;
            }

            case ExplorationHelper::State::LEFT:
            {
                if (fabs(-M_PI_2 - angleToTarget) > ANGLE_EPSILON) {
                    if ((-M_PI_2 - angleToTarget) < 0.0) {
                        explorer.rotate(ANGULAR_SPEED);
                    } else {
                        explorer.rotate(-ANGULAR_SPEED);
                    }
                    ROS_INFO("Angle %f", angleToTarget);
                } else {
                    if (explorer.getFrame() < FRAME_LIMIT) {
                        explorer.moveForward(LINEAR_SPEED);
                        explorer.incrementFrame();
                    } else {
                        if (explorer.getRightRange() > DISTANCE_EPSILON) {
                            explorer.setState(ExplorationHelper::State::FORWARD);
                        } else if (explorer.getFrontRange() > DISTANCE_EPSILON) {
                            explorer.moveForward(LINEAR_SPEED);
                        } else if (explorer.getLeftRange() > DISTANCE_EPSILON) {
                            explorer.setState(ExplorationHelper::State::BACK);
                        } else {
                            explorer.setState(ExplorationHelper::State::RIGHT);
                        }
                    }
                }
                break;
            }

            case ExplorationHelper::State::RIGHT:
            {
                if (fabs(M_PI_2 - angleToTarget) > ANGLE_EPSILON) {
                    if ((M_PI_2 - angleToTarget) < 0.0) {
                        explorer.rotate(ANGULAR_SPEED);
                    } else {
                        explorer.rotate(-ANGULAR_SPEED);
                    }
                    ROS_INFO("Angle %f", angleToTarget);
                } else {
                    if (explorer.getFrame() < FRAME_LIMIT) {
                        explorer.moveForward(LINEAR_SPEED);
                        explorer.incrementFrame();
                    } else {
                        if (explorer.getLeftRange() > DISTANCE_EPSILON) {
                            explorer.setState(ExplorationHelper::State::FORWARD);
                        } else if (explorer.getFrontRange() > DISTANCE_EPSILON) {
                            explorer.moveForward(LINEAR_SPEED);
                        } else if (explorer.getRightRange() > DISTANCE_EPSILON) {
                            explorer.setState(ExplorationHelper::State::BACK);
                        } else {
                            explorer.setState(ExplorationHelper::State::LEFT);
                        }
                    }
                }
                break;
            }

            case ExplorationHelper::State::BACK:
            {
                if (fabs(M_PI - fabs(angleToTarget)) > ANGLE_EPSILON) {
                    if ((M_PI - fabs(angleToTarget)) < 0.0) {
                        explorer.rotate(ANGULAR_SPEED);
                    } else {
                        explorer.rotate(-ANGULAR_SPEED);
                    }
                    ROS_INFO("Angle %f", angleToTarget);
                } else {
                    if (explorer.getLeftRange() > DISTANCE_EPSILON) {
                        explorer.setState(ExplorationHelper::State::RIGHT);
                    } else if (explorer.getRightRange() > DISTANCE_EPSILON) {
                        explorer.setState(ExplorationHelper::State::LEFT);
                    } else {
                        explorer.moveForward(LINEAR_SPEED);
                    }
                }
                break;
            }

            case ExplorationHelper::State::FINISH:
                ROS_INFO("Finished exploration");
                break;
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
