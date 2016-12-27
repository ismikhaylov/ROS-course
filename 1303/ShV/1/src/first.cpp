
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <sstream>


int main(int argc, char **argv)
{
 // Initiate new ROS node named "talker"
 ros::init(argc, argv, "pubTsim");
 ros::NodeHandle n;

 ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
 ros::Rate loop_rate(1); 

   int count = 0;
     srand (time(NULL));
   while (ros::ok()) 
   {

   geometry_msgs::Twist vel_msg;
   vel_msg.linear.x =(double)(rand() % 10 +1)/4.0;
   vel_msg.angular.z =(double)(rand() % 10 - 5)/2.0;


   velocity_publisher.publish(vel_msg);
      // ros::spinOnce(); 

      loop_rate.sleep();
       count++;
   }
   return 0;
}

