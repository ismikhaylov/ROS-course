#include <ros/ros.h>
#include <ctime>
#include <lab2_transport/Message.h>

std::string generate_message() {
    char *s = new char[5];
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
    for (int i = 0; i < 5; i++) {
        s[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
    }
    return std::string(s);
}

int main(int argc, char **argv)
{
  std::srand(0);
  ros::init(argc, argv, "lab2_scout");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<lab2_transport::Message>("/lab2", 1);
  ros::Rate loop_rate(10);
  
  ROS_INFO("Scout is ready to send messages");
  while(ros::ok()) {
    lab2_transport::Message command;
    std::string m = generate_message();
    int v = rand();
    command.message = m;
    command.value = v;
    ROS_INFO("New one: %s with number: %d", m.c_str(), v);
    pub.publish(command);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}