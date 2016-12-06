#include "Robot.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

std::string Killer::killerModel = "/home/elena/.gazebo/models/youbot/model.sdf";
std::string Target::activeModel = "/home/elena/.gazebo/models/box_target_red/model.sdf";
std::string Target::simpleModel = "/home/elena/.gazebo/models/box_target_green/model.sdf";
double Target::size = 0.2;
unsigned int Target::idCounter = 0;

Robot::Robot(geometry_msgs::Pose pos, const std::string &model, const ros::NodeHandle &handler,
			 const std::string &name) {
	nodeHandler = handler;
    this->model = model;
    modelName = name;
    position = pos;
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::service::waitForService("gazebo/delete_model");
    addModel();
    posePublisher = 
            nodeHandler.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    sleep(1.0);
 	setPosition(pos);
}

void Robot::addModel() {
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
}

void Robot::removeModel() {
    ros::ServiceClient delete_robot = 
             nodeHandler.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");
    gazebo_msgs::DeleteModel srv;
    srv.request.model_name = modelName;
    delete_robot.call(srv);
}

void Robot::changeModel(const std::string &newModel) {
    this->model = newModel;
    removeModel();
    addModel();
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

Killer::Killer(geometry_msgs::Pose pos, const ros::NodeHandle &handler) : 
    Robot(pos, killerModel, handler, "killer") {}

Target::Target(geometry_msgs::Pose pos, const ros::NodeHandle &handler, 
               const Borders &borders, const Speed &speed) : 
    Robot(pos, simpleModel, handler, "target" + std::to_string(idCounter)), 
    borders(borders), speed(speed) {
        id = idCounter;
        idCounter++;
    }


void Target::makeStep() {
    geometry_msgs::Pose newPose;
    double newY = position.position.y + speed.y;
    double newZ = position.position.z + speed.z;
    if (newY >= borders.rightBorder - Target::size/2 || newY <= borders.leftBorder + Target::size / 2) {
        speed.y *= -1;
        newY = position.position.y + speed.y;
    }
    if (newZ >= borders.upBorder - Target::size / 2 || newZ <= borders.bottomBorder + Target::size / 2) {
        speed.z *= -1;
        newZ = position.position.z + speed.z;
    }
    newPose.position.y = newY;
    newPose.position.z = newZ;
    newPose.position.x = position.position.x;
    newPose.orientation = position.orientation;
    setPosition(newPose);
}

unsigned int Target::getId() const {
    return id;
}

void Target::activate(bool active) {
    if (active) {
        changeModel(Target::activeModel);
    } else {
        changeModel(Target::simpleModel);
    }
}
