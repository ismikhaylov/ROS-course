#include "ros/ros.h"
#include "robot.h"
 
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "cleaner");

    Robot* mR = new Robot("missing");

    sleep(1.0);

    while (ros::ok()) {
                   
       mR->move(10);
       mr->rotate(3);
       
    }
    return 0;
}