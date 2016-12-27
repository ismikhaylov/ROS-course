#include <random>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <robots/succeed.h>

visualization_msgs::Marker createMarker(int id, float x, float y, float scale, float r, float g, float b)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "forest";
    marker.header.stamp = ros::Time::now();
    marker.ns = "robot_rescuer";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;

    marker.pose.orientation.w = 1.0;

    marker.id = id;
    marker.type = visualization_msgs::Marker::POINTS;

    marker.scale.x = scale;
    marker.scale.y = scale;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0f;

    marker.points.push_back(geometry_msgs::Point());

    return marker;
}

enum class State { Chase, Convoy, Waiting, Succeed };

bool missionComplete = false;

void succeedCallback(const robots::succeed::ConstPtr& message)
{
    if (!message->is_hooked)
    {
        missionComplete = true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_rescuer");
    ros::NodeHandle node;
    ros::Publisher rvizPublisher = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher succeedPublisher = node.advertise<robots::succeed>("/succeed", 1000);
    ros::Subscriber succeedSupscriber = node.subscribe("/succeed", 10, &succeedCallback);
    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_real_distribution<> uniform_dist(-6.0, 6.0);

    double dx, dy, distance;
    double hookRange = 0.05;

    double exitX, exitY;
    double robotX, robotY;

    exitX = robotX = uniform_dist(engine);
    exitY = robotY = uniform_dist(engine);

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    tf::Transform rescuer_transform;
    tf::StampedTransform stray_robot_transform;

    State state = State::Chase;

    visualization_msgs::Marker exitPoint = createMarker(0, exitX, exitY, 0.2f, 0.0f, 1.0f, 0.0f);
    visualization_msgs::Marker robot;

    double maxSpeed = 0.05;
    double angularSpeed = 1.2;
    double linearSpeed = 0.1;
    double currentAngularSpeed, currentLinearSpeed;
    double epsilon = 0.01;

    ros::Rate r(30);
    ROS_INFO("start");
    while (ros::ok())
    {
        robot = createMarker(1, robotX, robotY, 0.5f, 1.0f, 0.0f, 0.0f);
        // broadcast pose
        rescuer_transform.setOrigin(tf::Vector3(robotX, robotY, 0.0));
        rescuer_transform.setRotation(tf::Quaternion(robot.pose.orientation.x, robot.pose.orientation.y,
                                             robot.pose.orientation.z, robot.pose.orientation.w));
        broadcaster.sendTransform(tf::StampedTransform(rescuer_transform, ros::Time::now(), "world", "robot_rescuer"));
        if (state == State::Chase)
        {
            // listen stray pose
            listener.waitForTransform("world", "stray_robot", ros::Time::now(), ros::Duration(1.0));
            try {
                ros::Time commonTime;
                std::string error;
                listener.getLatestCommonTime("world", "stray_robot",commonTime, &error);
                listener.lookupTransform("world", "stray_robot", commonTime, stray_robot_transform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                rvizPublisher.publish(exitPoint);
                rvizPublisher.publish(robot);
                ros::Duration(1.0).sleep();
                continue;
            }
            // move rescuer robot to stray
            dx = stray_robot_transform.getOrigin().getX() - robotX;
            dy = stray_robot_transform.getOrigin().getY() - robotY;
            distance = std::sqrt(dx * dx + dy * dy);
            if (distance <= hookRange)
            {
                ROS_INFO("Hooked");
                robots::succeed succeedMessage;
                succeedMessage.is_hooked = true;
                succeedPublisher.publish(succeedMessage);
                state = State::Convoy;
            }
            currentLinearSpeed = linearSpeed * distance;
            currentAngularSpeed = angularSpeed * atan2(dy, dx);
            if (currentLinearSpeed > maxSpeed) currentLinearSpeed = maxSpeed;
            robotX += currentLinearSpeed * std::cos(currentAngularSpeed);
            robotY += currentLinearSpeed * std::sin(currentAngularSpeed);
        }
        else if (state == State::Convoy)
        {
            // move rescuer robot to exit
            dx = exitX - robotX;
            dy = exitY - robotY;
            distance = std::sqrt(dx * dx + dy * dy);
            if (distance <= epsilon)
            {
                ROS_INFO("Waiting...");
                robots::succeed succeedMessage;
                succeedMessage.is_hooked = true;
                succeedPublisher.publish(succeedMessage);
                state = State::Waiting;
            }
            else
            {
                currentLinearSpeed = linearSpeed * distance;
                currentAngularSpeed = angularSpeed * atan2(dy, dx);
                if (currentLinearSpeed > maxSpeed) currentLinearSpeed = maxSpeed;
                robotX += currentLinearSpeed * std::cos(currentAngularSpeed);
                robotY += currentLinearSpeed * std::sin(currentAngularSpeed);
            }
        }
        else if (state == State::Waiting)
        {
            ros::spinOnce();
            if (missionComplete)
            {
                ROS_INFO("Exit");
                state = State::Succeed;
            }
        }
        rvizPublisher.publish(exitPoint);
        rvizPublisher.publish(robot);
        r.sleep();
    }
}
