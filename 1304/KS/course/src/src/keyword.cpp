#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int16.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "keyword_control");
    system( "clear" );
    ros::NodeHandle nodeHandle;
    ros::Publisher pub = nodeHandle.advertise<std_msgs::Int16>("scout", 1000);
    ros::Rate loop(1);

    char key = 0x00;
    ros::Rate rate(10);

    std_msgs::Int16 msg;

    while (ros::ok() && key != 'e')
    {
        std::cout << "Press 's' to stop" << std::endl;
        std::cout << "Press 'c' to continue" << std::endl;
        std::cout << "Press 'e' to exit" << std::endl;
        std::cin >> key;

        system( "clear" );
        msg.data = 0;
        if (key == 's') {
            msg.data = 1;
            std::cout << "Send signal stop..." << std::endl << " -------------------- " << std::endl;
        } else if (key == 'c') {
            msg.data = 2;
            std::cout << "Send signal continue..." << std::endl << " -------------------- " << std::endl;
        } else if (key == 'e') {
            msg.data = 3;
            std::cout << "Send signal exit..." << std::endl << " -------------------- " << std::endl;
        }
        pub.publish( msg );

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}