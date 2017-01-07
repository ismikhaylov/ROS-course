#include <ros/ros.h>
#include "Robot.h"
using namespace std;

class Searcher : public Robot {

public:
	Searcher(ros::NodeHandle &robotNode, string name, string lostRobotName) : Robot(robotNode, name) {
		this->lostRobotName = lostRobotName;
		id = 2;
		x = y = z = 0;
		pub = robotNode.advertise<std_msgs::String>("/finderTopic", 1000);
	}
	void findLostRobot()
	{
		set_shape("search");
		moveTo(name, lostRobotName);
		pubMsgToLostRobot(name);	
	}

	void leadToHome() {
		set_shape("leads");
		moveTo(name, "world");
		//set_shape("home");
		//updateMarker();
		pubMsgToLostRobot("We're home");
	}

	void initPaintSearcher()
	{
		sendPos("Finder robot");
		updateMarker();
		sleep(3);
	}

private:
	string lostRobotName;
	ros::Publisher pub;

	void moveTo(const std::string& targetFrame, const std::string& sourceFrame) {
		tf::TransformListener listener;

		ros::Rate rate(15);
		while (robotNode.ok()) {
			tf::StampedTransform transform;

			try {
				listener.lookupTransform(targetFrame , sourceFrame , ros::Time(0), transform);
			}
			catch (tf::TransformException &e) {
				sendPos("Finder robot");
				ros::Duration(1.0).sleep();
				continue;
			}
			
			cout << "Distance to " << sourceFrame << " = " << sqrt(pow(fabs(transform.getOrigin().x()),2) + pow(fabs(transform.getOrigin().y()),2)) << endl;
			
			if (fabs(transform.getOrigin().x()) <= DX && fabs(transform.getOrigin().y()) <= DY)
				break;

			x += sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().x(), 2)) / 2.0 * (transform.getOrigin().x() < 0) ? -1.0 : 1.0;
			y += sqrt(pow(transform.getOrigin().y(), 2) + pow(transform.getOrigin().y(), 2)) / 2.0 * (transform.getOrigin().y() < 0) ? -1.0 : 1.0;
			
			sendPos("Finder robot");
			updateMarker();
			
			rate.sleep();
		}
	}
	void pubMsgToLostRobot(const std::string& msg)
	{
		std_msgs::String s;
		int i = 0;
		s.data = msg;
		cout << "Sended message \"" << msg << "\"" << endl;
	  	ros::Rate loop_rate(1);
	  	while (i <= 3) {
	  		i++;
	        pub.publish(s);
	        loop_rate.sleep();
	    }

	}

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "finder_robot");
	ros::NodeHandle finder_robot_node;
	
	Searcher searcher(finder_robot_node, "finder", "missing");
	searcher.initVisualisation();
	searcher.initPaintSearcher();
	searcher.findLostRobot();
	searcher.leadToHome();
	return 0;
}