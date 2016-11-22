#include "ros/ros.h"
#include "std_msgs/String.h"
#include "message/location.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "writer");
	ros::NodeHandle w;

	ros::Publisher writer = w.advertise<message::location>("coordinates", 1000);

	message::location coords;
	coords.x = 5;
	coords.y = 6;
	coords.z = 14;
	while (ros::ok() )
	{
		ROS_INFO("Rocket launched to coordinates: x= %f y= %f z= %f\n",coords.x,coords.y,coords.z);
		writer.publish(coords);
		ros::spinOnce();
		sleep(3);
	}
	return 0;
}