// This header defines the standard ROS classes .
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
   // Initialize the ROS system.
  ros::init(argc, argv, "lab1");
  // Establish this program as a ROS node.
  ros::NodeHandle nh;
  // Send message to turtlesim with size_buffer = 1000
  ros::Publisher publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
  ros::Rate loop_rate(1);

  geometry_msgs::Twist command;
  
  command.angular.z = 0.0;
  command.linear.x = 2.0;
  float temp;
  int count = 0;

  while(ros::ok() && count < 39) {
    //Swap angular.z and linear.x
    temp = command.angular.z;
    command.angular.z = command.linear.x;
    command.linear.x = temp;
    //Publish message
    publisher.publish(command);
    ros::spinOnce();
    loop_rate.sleep();
    //Sending output  to draw a triangle number
    if(count % 6 == 0 && count != 0) 
      ROS_INFO(" %d triangle complete! \n", (int)(count / 6));
    //Increment count
    count++;   
  }

  return 0;
}
