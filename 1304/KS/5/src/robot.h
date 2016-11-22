#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;

#define STEPS 40
#define DXY 0.1
#define R 3

class Robot
{
public:
	Robot(const char* name);

	void move(float x, float y);
	void move(float x, float y, float distance);
	void moveToAnotherRobot(float distance = 0);

	bool checkPosAnotherRobot(const char* tf);
	bool isPosition(float x, float y);

protected:
	const char* _name;
	float _aRX;
	float _aRY;

	ros::NodeHandle _nh;
	ros::Publisher _pub;
	gazebo_msgs::ModelState _msg;
};