#include <ros/ros.h>
#include "Robot.h"
using namespace std;

//Class lost robot, 
class LostRobot : public Robot {
public:
	//Constructor
	LostRobot(ros::NodeHandle &robotNode, string name) : Robot(robotNode, name) {
		this->rdyToFollow = false;
		id = 1;
		x = y = z = 0.0;
	}

	//Handler function, processing received messages from finder robot
	void msg_handler(const std_msgs::String& str) {
		if (str.data != "We're home") {
			rdyToFollow = true;
			finderRobotName = str.data;
		} else 
		{
			rdyToFollow = false;
			cout << "I took a message \"" << str.data << "\"" << endl;
		}

		
	}


	void getLost() {
		//Set shape and color of the lost robot
		set_shape("lost");
		ros::Rate loop_rate(15);
		srand (time(NULL));
		//Move to a random side until it finds a finder robot
		while(ros::ok() && !rdyToFollow) {
			x += (double) ((rand() % 9) - 4)/2;
			y += (double) ((rand() % 9) - 4)/2;
			z = 0.0;
			sendPos("Lost robot");
			updateMarker(); 
			ros::spinOnce();
			loop_rate.sleep();
		}
		// Then follow finder robot
		set_shape("follow");
		followToFinderRobot();
		//set_shape("home");
		z = 1.0;
		updateMarker();
	}
// Listen messages from finder fobot
	ros::Subscriber listenFinderRobot() {
		return robotNode.subscribe("/finderTopic", 1000, &LostRobot::msg_handler, this);
	}

private:
	string finderRobotName;
	bool rdyToFollow;
	

	void followToFinderRobot() {
		tf::TransformListener listener;
		ros::Rate rate(15);
		x = (int)x;
		y = (int)y;
		cout << "Lost robot found and starts to follow the finder robot!" << endl;
		while (robotNode.ok() && rdyToFollow) {
			tf::StampedTransform transform;
			try{
				listener.lookupTransform(name, finderRobotName, ros::Time(0), transform);
			}
			catch (tf::TransformException &e) {
				sendPos("Lost robot");
				ros::Duration(1.0).sleep();
				continue;
			}
			// Calculate next position
			x += (int)(transform.getOrigin().x());
			y += (int)(transform.getOrigin().y());

			sendPos("Lost robot");
			updateMarker();			
			ros::spinOnce();
			rate.sleep();
		}
	}


};

int main(int argc, char **argv) {

	ros::init(argc, argv, "lost_robot");
	ros::NodeHandle lost_robot_node;
	LostRobot lost_robot(lost_robot_node, "missing");
	lost_robot.initVisualisation();
	ros::Subscriber sub = lost_robot.listenFinderRobot();
	lost_robot.getLost();

	return 0;
}