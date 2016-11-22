#include "Robot.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

Robot::Robot(geometry_msgs::Pose pos, const std::string &model, const ros::NodeHandle &handler,
			 const std::string &name) {
	nodeHandler = handler;
    this->model = model;
    modelName = name;
    position = pos;
    saved = false;
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot = 
             nodeHandler.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;

    std::ifstream fin(this->model);
    std::string modelText;
    std::string buffer;
    while(!fin.eof()){
        getline(fin, buffer);
        modelText += buffer + "\n";
    }
    srv.request.model_xml = modelText;
    srv.request.model_name = modelName;
    srv.request.initial_pose = position;
    add_robot.call(srv);
 
    posePublisher = 
            nodeHandler.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    sleep(1.0);
 	setPosition(pos);
}

geometry_msgs::Pose Robot::getPosition() {
    return position;
}

void Robot::broadcastPosition() {
    // Broadcast new position
    static tf::TransformBroadcaster broadcaster;
    std::string posTopic = "/" + modelName + "_pos";
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(position.position.x, position.position.y, position.position.z) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", posTopic));
}

void Robot::setPosition(const geometry_msgs::Pose &pos) {
	gazebo_msgs::ModelState msg;
	position = pos;
    msg.model_name = modelName;
    msg.pose = pos;
    posePublisher.publish(msg);
}

void Robot::run(const geometry_msgs::Pose &pos, bool freq) {
	geometry_msgs::Pose newPose;
	double angleCoef = (pos.position.y - position.position.y) / (pos.position.x - position.position.x);
	double angle = atan(angleCoef);
	if (pos.position.x < position.position.x) {
		angle += M_PI;
	}
	tf::Quaternion q(tf::Vector3(0, 0, 1), angle);
	geometry_msgs::Quaternion odom_quat;
  	tf::quaternionTFToMsg(q, odom_quat);
	newPose.orientation = odom_quat;
	int value; 
	if (freq) {
		value = 50;
	} else {
		value = 10;
	}
	float stepX = float(pos.position.x - position.position.x) / value;
	float stepY = float(pos.position.y - position.position.y) / value;
	ros::Rate rate(5);
	for (int step = 0; step < value && ros::ok(); step++) {
		float newX = position.position.x + stepX;
		float newY = position.position.y + stepY;
		newPose.position.x = newX;
		newPose.position.y = newY;
		setPosition(newPose);
		rate.sleep();
		broadcastPosition();
	}
} 

void SimpleRobot::follow(const std_msgs::String &msg) {
    saved = true;
    std::string rescure = msg.data;
    ROS_INFO_STREAM(rescure);
    std::string currentTopic = "/" + modelName + "_pos";
    std::string posTopic = "/" + rescure + "_pos";
    listener.waitForTransform(currentTopic, posTopic, ros::Time(0), ros::Duration(1));

    ros::Rate rate(20);

    while (ros::ok()) {
        tf::StampedTransform transform;

        try {
            listener.lookupTransform(currentTopic, posTopic, ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
            ros::Duration(1).sleep();
            continue;
        }
        geometry_msgs::Pose pose;
        pose.position.x = position.position.x + transform.getOrigin().x() - 0.35;
        pose.position.y = position.position.y + transform.getOrigin().y() - 0.35;
        run(pose);
        rate.sleep();
    }
}

bool Robot::isSaved() {
    return saved;
}

std::string  SimpleRobot::simpleRobotModel = "/home/elena/.gazebo/models/pioneer2dx/model.sdf";
SimpleRobot::SimpleRobot(geometry_msgs::Pose pos, const ros::NodeHandle &handler) : 
    Robot(pos, simpleRobotModel, handler, "simple_robot") {
        const int queueSize = 10;
        publisher = nodeHandler.advertise<std_msgs::String>("help_msgs", queueSize);
        helpSubscriber = nodeHandler.subscribe("instructions", 10, &SimpleRobot::follow, this);
        saved = false;
    }

void SimpleRobot::sendHelpMessage() {
    std_msgs::String msg;
    msg.data = modelName;
    publisher.publish(msg);
    ros::Rate rate(1);
    rate.sleep();
    ROS_INFO_STREAM("send");
    publisher.publish(msg);
}

std::string RescuerRobot::rescuerRobotModel = "/home/elena/.gazebo/models/pioneer3at/model.sdf";
RescuerRobot::RescuerRobot(geometry_msgs::Pose pos, const ros::NodeHandle &handler) : 
    Robot(pos,rescuerRobotModel, handler, "rescuer_robot"), entryPosition(pos) {
        helpSubscriber = nodeHandler.subscribe("help_msgs" , 10, &RescuerRobot::findLost, this);
    }

bool RescuerRobot::hasFound(float posX, float posY) {
    return (fabs(posX) < 0.15 && fabs(posY) < 0.15);
}

void RescuerRobot::findLost(const std_msgs::String &msg) {
	broadcastPosition();
    std::string lostName = msg.data;
    std::string currentTopic = "/" + modelName + "_pos";
    ROS_INFO_STREAM(lostName);
    publisher = nodeHandler.advertise<std_msgs::String>("instructions", 10);
    std::string posTopic = "/" + lostName + "_pos";
    listener.waitForTransform(currentTopic, posTopic, ros::Time(0), ros::Duration(1));
    ros::Rate rate(20);
    while (ros::ok()) {
        tf::StampedTransform transform;
        broadcastPosition();
        try {
            listener.lookupTransform(currentTopic, posTopic, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1).sleep();
            continue;
        }

        if (hasFound(transform.getOrigin().x(), transform.getOrigin().x())) {

            std_msgs::String msg;
            ROS_INFO_STREAM("send");
            msg.data = modelName;

            publisher.publish(msg);
            ros::Duration(1).sleep();
            returnBack();
            return;
        }
        geometry_msgs::Pose pose;
        pose.position.x = position.position.x + transform.getOrigin().x()/5;
        pose.position.y = position.position.y + transform.getOrigin().y()/5;
        run(pose, false);
        rate.sleep();
    }
}

void RescuerRobot::returnBack() {
    run(entryPosition);
}