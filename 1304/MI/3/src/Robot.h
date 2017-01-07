#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <ros/console.h>
#include <string> 
#include <cmath>  
using namespace std;

#define DX 0.5f
#define DY 0.5f
class Robot
{
public:
	Robot(ros::NodeHandle &robotNode, string name);

	void initVisualisation();
protected:
	string name;
	ros::NodeHandle &robotNode;
	int id;
	double x, y, z;

	ros::Publisher marker_pub;
	visualization_msgs::Marker marker;	

	void sendPos(string typeRobot);

	void updateMarker();
	void set_shape(string currState);

};