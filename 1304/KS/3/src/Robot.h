#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define STEPS 10
#define R 1

class Robot
{
public:
	Robot(const char* name, const int id);

	void move(double x, double z);

	bool checkPosAnotherRobot(const char* tf);
	void moveToAnotherRobot();

protected:
	const char* _name;
	float _aRX;
	float _aRY;
	ros::NodeHandle _nh;
	ros::Publisher _markerPub;
	visualization_msgs::Marker _marker;
};