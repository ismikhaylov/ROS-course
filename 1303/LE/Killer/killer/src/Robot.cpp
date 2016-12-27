#include "Robot.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include <regex>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

std::string Killer::killerModel = "/home/elena/.gazebo/models/youbot/model.sdf";
std::string Target::activeModel = "/home/elena/.gazebo/models/box_target_red/model.sdf";
std::string Target::simpleModel = "/home/elena/.gazebo/models/box_target_green/model.sdf";
std::string Bullet::bulletModel = "/home/elena/.gazebo/models/bullet/model.sdf";
std::string Barrier::barrierModel = "/home/elena/.gazebo/models/box/model.sdf";
double Target::size = 0.2;
unsigned int Target::idCounter = 0;
unsigned int Barrier::idCounter = 0;

Robot::Robot(geometry_msgs::Pose pos, const std::string &model, ros::NodeHandle &handler,
			 const std::string &name) {
	nodeHandler = handler;
    this->model = model;
    modelName = name;
    position = pos;
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::service::waitForService("gazebo/delete_model");
    addModel();
    posePublisher = 
            nodeHandler.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1);
    sleep(1.0);
 	setPosition(pos);
}

Robot::Robot(geometry_msgs::Pose pos, const std::string &model, ros::NodeHandle &handler,
             const std::string &name, double sizeX, double sizeY, double sizeZ):
             sizeX(sizeX), sizeY(sizeY), sizeZ(sizeZ) {
    nodeHandler = handler;
    this->model = model;
    modelName = name;
    position = pos;
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::service::waitForService("gazebo/delete_model");
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
    size_t index = 0;
    std::regex sizeRegex("1 1 1");
    std::string sizeString = std::to_string(sizeX) + " " + std::to_string(sizeY) + " " + std::to_string(sizeZ);
    modelText = std::regex_replace(modelText, sizeRegex, sizeString);
    srv.request.model_xml = modelText;
    srv.request.model_name = modelName;
    srv.request.initial_pose = position;
    add_robot.call(srv);
    posePublisher = 
            nodeHandler.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1);
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

}

void Target::broadcastPosition() {
    static tf::TransformBroadcaster broadcaster;
    if (isActive) {
        // Broadcast new position
        std::string posTopic = "/target_pos";
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(position.position.x, position.position.y, position.position.z) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", posTopic));
    }
}

bool Target::onCoordinate(float y, float z) {
    return (y >= position.position.y - Target::size / 2 && y <= position.position.y + Target::size / 2)
            && (z >= position.position.z - Target::size / 2 && z <= position.position.z + Target::size / 2);
}

bool Barrier::onCoordinate(float y, float z) {
    return (y >= position.position.y - sizeY / 2 && y <= position.position.y + sizeY / 2)
            && (z >= position.position.z - sizeZ / 2 && z <= position.position.z + sizeZ / 2);
}

void Killer::broadcastPosition() {
    static tf::TransformBroadcaster broadcaster;
    // Broadcast new position
    std::string posTopic = "/killer_pos";
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(position.position.x, position.position.y, position.position.z) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", posTopic));
}

bool Killer::hasFoundTargetY(float posY) {
    return (fabs(posY) < 0.25);
}

bool Killer::hasFoundTargetX(float posX) {
    return (fabs(posX) <= firingRange);
}

void Killer::runY(float posY) {
    unsigned int maxStepNums = 5;
    int direction = posY < position.position.y ? -1 : 1;  
    unsigned int stepsNum = fabs(posY)/speed.y;
    stepsNum = maxStepNums < stepsNum ? maxStepNums : stepsNum;
    stopY = false;
    if (stopX) {
        stepsNum = 1;
    }
    for (int i = 0; i < stepsNum; i++) {
        geometry_msgs::Pose newPose = position;
        newPose.position.y =  position.position.y + direction*speed.y;
        for (const auto& objects : firstObjects) {

            if (fabs(objects.first - newPose.position.x) < 0.3) {
                for (const auto& posVector : objects.second) {
                    for (const auto & pos : posVector) {

                        if (fabs(pos.position.y - newPose.position.y) < 0.3) {
                            stopY = true;
                            
                        }
                    }
                }
            }
        }
        if (!stopY) {
            setPosition(newPose);
            ros::Duration(0.2).sleep();
            broadcastPosition();
        } else {
            break;
        }
    }
}

void Killer::runX(float posX) {
    unsigned int maxStepNums = 5;
    int direction = posX < position.position.x ? -1 : 1;
    unsigned int stepsNum = (fabs(posX)- firingRange)/speed.x;
    stepsNum = maxStepNums < stepsNum ? maxStepNums : stepsNum;
    stopX = false;
    if (stopY) {
        stepsNum = 1;
    }

    for (int i = 0; i < stepsNum; i++) {
        geometry_msgs::Pose newPose = position;
        newPose.position.x =  position.position.x + direction*speed.x;
        for (const auto& objects : firstObjects) {

            if (fabs(objects.first - newPose.position.x) < 0.5 && 
                fabs(objects.first - newPose.position.x) > 0.05) {
                for (const auto& posVector : objects.second) {
                    for (const auto & pos : posVector) {
                        if (fabs(pos.position.y - newPose.position.y) < 0.3) {
                            stopX = true;

                        }
                    }
                }
            }
        }
        if (!stopX) {
            setPosition(newPose);
            ros::Duration(0.2).sleep();
            broadcastPosition();
        } else {
            break;
        }
    }
}

void Killer::analyzeScan(const laser_scan::LaserScan& msg) {
    analyzeScanObjects(msg, !simpleLaserScan);
}

void Killer::analyzeScanObjects(const laser_scan::LaserScan& msg, bool dynamic) {
    
    std::map < float, std::vector<std::vector<geometry_msgs::Pose> > > objects;
    std::vector<geometry_msgs::Pose> object;
    int z_size = msg.z_size;
    int z_index;
    int y_index;
    float z_max = msg.z_max;
    if (!dynamic) {
        z_max = 0.05;
    }

    for (float z = msg.z_min, z_index = 0; z <= z_max; z+= msg.z_increment, z_index++) {

        int last_y = -10;
        for (float y = msg.y_min, y_index = 0; y <= msg.y_max; y+= msg.y_increment, y_index++) {
            if (msg.ranges[z_index * z_size + y_index] < std::numeric_limits<float>::infinity()) {
                geometry_msgs::Pose newPose;
                newPose.position.x = msg.ranges[z_index * z_size + y_index];
                newPose.position.y = y;
                newPose.position.z = z;

                if (y_index == last_y + 1) {              
                    object.push_back(newPose);
                    
                } else {
                    if (!object.empty()) {
                        if (objects.find(object[0].position.x) == objects.end()) {
                            objects[object[0].position.x] = std::vector<std::vector<geometry_msgs::Pose> >();
                        }
                        objects[object[0].position.x].push_back(object);
                    }
                    object.clear();
                    object.push_back(newPose);
                }
                last_y = y_index;
            }
        }
    }
    if (!object.empty()) {
        if (objects.find(object[0].position.x) == objects.end()) {
            objects[object[0].position.x] = std::vector<std::vector<geometry_msgs::Pose> >();
        }
        objects[object[0].position.x].push_back(object);
    }
    if (dynamic) {
        if (firstTime == -1) {
            firstTime = msg.time;
            firstObjects = objects;
        } else if (secTime == -1) {
            secTime = msg.time;
            secObjects = objects;
        } else {
            firstTime = -1;
            secTime = -1;
        }
    } else {
        firstObjects = objects;
    }
}

void Killer::calculateTargetSpeed() {
    std_msgs::String msg;
    msg.data = "laser_scan";
    simpleLaserScan = false;

    publisher.publish(msg);
    ros::Duration(1).sleep();
    ros::spinOnce();
    publisher.publish(msg);
    ros::Duration(1).sleep();
    ros::spinOnce();

    if (firstTime != -1 && secTime != -1) {

        if (firstObjects.find(targetDistance) != firstObjects.end() &&
            secObjects.find(targetDistance) != secObjects.end()) {
            if (!fireCommand) {
                ros::Duration(0.5).sleep();
            }
            if (fireCommand) {

                int lineNumStart = 0;
                int colNumStart = 0;
                int firstLineNumMiddle = firstObjects[targetDistance].size()/2;
                int firstColNumMiddle = firstObjects[targetDistance][firstLineNumMiddle].size() / 2;
                int firstLineNumEnd = firstObjects[targetDistance].size() - 1;
                int firstColNumEnd = firstObjects[targetDistance][firstLineNumEnd].size() - 1;
                geometry_msgs::Pose firstStart = firstObjects[targetDistance][lineNumStart][colNumStart];
                geometry_msgs::Pose firstMiddle = firstObjects[targetDistance][firstLineNumMiddle][firstColNumMiddle];
                geometry_msgs::Pose firstEnd = firstObjects[targetDistance][firstLineNumEnd][firstColNumEnd];

                int secLineNumMiddle = secObjects[targetDistance].size()/2;
                int secColNumMiddle = secObjects[targetDistance][secLineNumMiddle].size() / 2;
                int secLineNumEnd = firstObjects[targetDistance].size() - 1;
                int secColNumEnd = firstObjects[targetDistance][secLineNumEnd].size() - 1;

                geometry_msgs::Pose secondStart = firstObjects[targetDistance][lineNumStart][colNumStart];
                geometry_msgs::Pose secondMiddle = firstObjects[targetDistance][secLineNumMiddle][secColNumMiddle];
                geometry_msgs::Pose secondEnd = firstObjects[targetDistance][secLineNumEnd][secColNumEnd];
                if (secObjects.size() < firstObjects.size()) {
                    lineNum = secObjects[targetDistance].size()/2;
                    colNum = secObjects[targetDistance][lineNum].size() / 2;
                }
                double timeIncr = secTime - firstTime;
                double speedY = (secondStart.position.y - firstStart.position.y) / timeIncr;
                geometry_msgs::Pose second = secObjects[targetDistance][lineNum][colNum];
                double speedZ = (secondStart.position.z - firstStart.position.z) + 
                (secondMiddle.position.z - firstMiddle.position.z ) + 
                (secondEnd.position.z - firstEnd.position.z)) / timeIncr;
                geometry_msgs::Pose second = secObjects[targetDistance][lineNum][colNum];
                double speedY = (second.position.y - first.position.y) /timeIncr;
                double speedZ = (second.position.z - first.position.z) /timeIncr;
                geometry_msgs::Pose bulletPos;
                bulletPos.position.x  = position.position.x - 0.2;
                bulletPos.position.y = position.position.y;
                bulletPos.position.z = 0.5;
                Speed bulletSpeed;
                bulletSpeed.x = 0.1;
                double flyTime = fabs(bulletPos.position.x - targetDistance) / (bulletSpeed.x / 0.1);
                double yDistance = flyTime * speedY;
                double zDistance = flyTime * speedZ;
                double yDistanceBullet = secondStart.position.y + yDistance - bulletPos.position.y;
                double zDistanceBullet = secondStart.position.z + zDistance - bulletPos.position.z;
                bulletSpeed.y = yDistanceBullet / flyTime * 0.1;
                bulletSpeed.z = zDistanceBullet / flyTime * 0.1;
                Bullet bullet(bulletPos, nodeHandler, bulletSpeed);
                firstTime = -1;
                secTime = -1;
                firstObjects.clear();
                secObjects.clear();
                bullet.fly(targetDistance, position.position.y + yDistanceBullet, position.position.z + zDistanceBullet);
                bullet.removeModel();
            }
        }
    }
}

void Bullet::fly(double x, double y, double z) {
    int directionX = x < position.position.x ? -1 : 1;
    int directionY = y < position.position.y ? -1 : 1;  
    while (fabs(position.position.x - x) > 0.05) {
        geometry_msgs::Pose newPose;
        newPose.position.x = position.position.x + directionX*speed.x;
        newPose.position.y = position.position.y + directionY*speed.y;
        newPose.position.z = position.position.z + speed.z;
        setPosition(newPose);
        ros::Duration(0.1).sleep();
    }
    publisher.publish(position);
}

void Killer::followTarget() {
    broadcastPosition();
    ros::Duration(0.1).sleep();
    std::string currentTopic = "/killer_pos";
    std::string posTopic = "/target_pos";
    listener.waitForTransform(currentTopic, posTopic, ros::Time(0), ros::Duration(1));

    while (ros::ok()) {
        tf::StampedTransform transform;
        broadcastPosition();
        try {
            std_msgs::String msg;
            msg.data = "simple_laser_scan";
            simpleLaserScan = true;
            publisher.publish(msg);
            ros::Duration(0.3).sleep();
            listener.lookupTransform(currentTopic, posTopic, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1).sleep();
            continue;
        }

        // Get laser scan.
        ros::spinOnce();
        while (hasFoundTargetX(transform.getOrigin().x())) {
            if (hasFoundTargetY(transform.getOrigin().y())) {
                targetDistance = position.position.x + transform.getOrigin().x();
                calculateTargetSpeed();
                return;
            }
            runY(position.position.y + transform.getOrigin().y());
            if (stopY) {
                runX(position.position.x + transform.getOrigin().x());
                while (stopX) {
                    int direction = transform.getOrigin().y() > 0 ? -1 : 1;
                    runY(position.position.y + direction*0.2);
                    broadcastPosition();
                    runX(position.position.x + transform.getOrigin().x());
                    std_msgs::String msg;
                    msg.data = "simple_laser_scan";
                    simpleLaserScan = true;
                    publisher.publish(msg);
                    ros::Duration(0.3).sleep();
                }
            }
            break;
        }
        if (!hasFoundTargetX(transform.getOrigin().x())) {
            runX(position.position.x + transform.getOrigin().x());
            if (stopX) {
                runY(position.position.y + transform.getOrigin().y());
                if (stopY) {
                    while (stopX) {
                        int direction = transform.getOrigin().y() > 0 ? -1 : 1;
                        runY(position.position.y + direction*0.2);
                        broadcastPosition();
                        runX(position.position.x + transform.getOrigin().x());
                        std_msgs::String msg;
                        msg.data = "simple_laser_scan";
                        simpleLaserScan = true;
                        publisher.publish(msg);
                        ros::Duration(0.3).sleep();
                    }
                }
                
            }
        }
    }
}

void Robot::setPosition(const geometry_msgs::Pose &pos) {
	gazebo_msgs::ModelState msg;
	position = pos;
    msg.model_name = modelName;
    msg.pose = pos;
    posePublisher.publish(msg);
}

Killer::Killer(geometry_msgs::Pose pos, ros::NodeHandle &handler, double firingRange) : 
    Robot(pos, killerModel, handler, "killer"), firingRange(firingRange) {
        const int queueSize = 1;
        publisher = nodeHandler.advertise<std_msgs::String>("get_laser_scan", queueSize);
        this->speed = {0.18, 0.18, 0};
        firstTime = -1;
        secTime = -1;
        scanSubscriber = nodeHandler.subscribe("/rrbot/laser/scan", 1, &Killer::analyzeScan, this);
        commandSubscriber = nodeHandler.subscribe("robot_fire", queueSize, &Killer::getCommand, this);
        fireCommand = false;
    }

void Killer::getCommand(const std_msgs::String &msg) {
    if (msg.data == "fire") {
        fireCommand = true;
    } 
}

Target::Target(geometry_msgs::Pose pos, ros::NodeHandle &handler, 
               const Borders &borders, const Speed &speed) : 
    Robot(pos, simpleModel, handler, "target" + std::to_string(idCounter)), 
    borders(borders) {
        id = idCounter;
        idCounter++;
        isActive = false;
        this->speed = speed;
    }

Barrier::Barrier(geometry_msgs::Pose pos, ros::NodeHandle &handler, double sizeX, double sizeY,
                 double sizeZ): Robot(pos, barrierModel, handler, "barrier" + std::to_string(idCounter),
                 sizeX, sizeY, sizeZ) {
    idCounter++;
}
                    
                            

Bullet::Bullet(geometry_msgs::Pose pos, ros::NodeHandle &handler,
           const Speed &speed) : Robot(pos, bulletModel, handler, "bullet") {
    const int queueSize = 1;
    publisher = nodeHandler.advertise<geometry_msgs::Pose>("bullet_pos", queueSize);
    this->speed = speed;
}

void Target::makeStep() {
    geometry_msgs::Pose newPose;
    double newY = position.position.y + speed.y;
    double newZ = position.position.z + speed.z;
    // Check barriers.
    bool collisionY = false;
    bool collisionZ = false;
    for (Barrier *barrier : barriers) {
        if (newY >= barrier->getPosition().position.y - barrier->getSizeY() / 2 ||
            newY <= barrier->getPosition().position.y + barrier->getSizeY() / 2) {
            if (position.position.z <= barrier->getHeight()) {
                collisionY = true;  
            } else if (newZ <= barrier->getHeight()) {
                collisionZ = true;
            }
            
        }
    }
    if (newY >= borders.rightBorder - Target::size/2 || newY <= borders.leftBorder + Target::size / 2 ||
        collisionY) {
        speed.y *= -1;
        newY = position.position.y + speed.y;
    }
    if (newZ >= borders.upBorder - Target::size / 2 || newZ <= borders.bottomBorder + Target::size / 2 ||
        collisionZ) {
        speed.z *= -1;
        newZ = position.position.z + speed.z;
    }
    newPose.position.y = newY;
    newPose.position.z = newZ;
    newPose.position.x = position.position.x;
    newPose.orientation = position.orientation;
    setPosition(newPose);
}

double Barrier::getSizeY() {
    return sizeY;
}
double Barrier::getSizeX() {
    return sizeX; 
}
double Barrier::getHeight() {
    return sizeZ;
}

unsigned int Target::getId() const {
    return id;
}

void Target::activate(bool active) {
    if (active) {
        isActive = true;
        changeModel(Target::activeModel);
    } else {
        isActive = false;
        changeModel(Target::simpleModel);
    }
}
