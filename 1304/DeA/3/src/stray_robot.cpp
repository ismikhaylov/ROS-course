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
    marker.ns = "stray_robot";
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

enum class State { Chase, Convoy, Succeed };

bool isHooked = false;

void succeedCallback(const robots::succeed::ConstPtr& message)
{
    if (message->is_hooked)
    {
        isHooked = true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stray_robot");
    ros::NodeHandle node;
    ros::Publisher rvizPublisher = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher succeedPublisher = node.advertise<robots::succeed>("/succeed", 1000);
    ros::Subscriber succeedSupscriber = node.subscribe("/succeed", 10, &succeedCallback);
    visualization_msgs::Marker robot;
    State state = State::Chase;

    double dx, dy, distance;
    double robotX, robotY;
    double goalX, goalY;

    double maxSpeed = 0.05;
    double angularSpeed = 1.2;
    double linearSpeed = 0.1;
    double currentAngularSpeed, currentLinearSpeed;
    double epsilon = 0.01;

    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_real_distribution<> uniform_dist(-6.0, 6.0);

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    tf::Transform stray_robot_transform;
    tf::StampedTransform rescuer_transform;

    robotX = uniform_dist(engine);
    robotY = uniform_dist(engine);

    goalX = uniform_dist(engine);
    goalY = uniform_dist(engine);

    ros::Rate r(30);
    ROS_INFO("start");
    while (ros::ok())
    {
        robot = createMarker(0, robotX, robotY, 0.5f, 0.0f, 0.0f, 1.0f);

        // broadcast pose
        stray_robot_transform.setOrigin(tf::Vector3(robotX, robotY, 0.0));
        stray_robot_transform.setRotation(tf::Quaternion(robot.pose.orientation.x, robot.pose.orientation.y,
                                             robot.pose.orientation.z, robot.pose.orientation.w));
        broadcaster.sendTransform(tf::StampedTransform(stray_robot_transform, ros::Time::now(), "world", "stray_robot"));

        if (state == State::Chase)
        {
            ros::spinOnce();
            // randomMove
            if (isHooked)
            {
                ROS_INFO("Is hooked");
                isHooked = false;
                state = State::Convoy;
            }
            else
            {
                //ROS_INFO("coords: (%f, %f)", robotX, robotY);
                dx = goalX - robotX;
                dy = goalY - robotY;
                distance = std::sqrt(dx * dx + dy * dy);
                if (distance <= epsilon)
                {
                    goalX = uniform_dist(engine);
                    goalY = uniform_dist(engine);
                }
                else
                {
                    currentLinearSpeed = linearSpeed * distance;
                    if (currentLinearSpeed > maxSpeed) currentLinearSpeed = maxSpeed;
                    currentAngularSpeed = angularSpeed * atan2(dy, dx);
                    robotX += currentLinearSpeed * std::cos(currentAngularSpeed);
                    robotY += currentLinearSpeed * std::sin(currentAngularSpeed);
                }
            }
        }
        else if (state == State::Convoy)
        {
            // listen stray pose
            listener.waitForTransform("world", "robot_rescuer", ros::Time::now(), ros::Duration(1.0));
            try {
                ros::Time commonTime;
                std::string error;
                listener.getLatestCommonTime("world", "robot_rescuer",commonTime, &error);
                listener.lookupTransform("world", "robot_rescuer", commonTime, rescuer_transform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                rvizPublisher.publish(robot);
                ros::Duration(1.0).sleep();
                continue;
            }
            // move stray robot to rescuer
            dx = rescuer_transform.getOrigin().getX() - robotX;
            dy = rescuer_transform.getOrigin().getY() - robotY;
            distance = std::sqrt(dx * dx + dy * dy);
            if (distance <= epsilon)
            {
                if (isHooked)
                {
                    robots::succeed succeedMessage;
                    succeedMessage.is_hooked = false;
                    succeedPublisher.publish(succeedMessage);
                    ROS_INFO("Exit");
                    state = State::Succeed;
                }
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
        rvizPublisher.publish(robot);
        r.sleep();
    }
}
