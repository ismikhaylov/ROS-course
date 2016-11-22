#include "ros/ros.h"
#include "robot.h"
 
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "lab5");

    Robot* mR = new Robot("missing");

    sleep(1.0);

    while (ros::ok()) {
        if (mR->checkPosAnotherRobot( "assistant" )) {
            mR->moveToAnotherRobot();
            if (mR->isPosition( -20, -20)) {
                ROS_INFO( "Missing robot end." );
                mR->move( rand() % 20, rand() % 20 );
            }
        } else {            
            mR->move( rand() % 10, rand() % 10, 5 );
        }
    }
 /*
    gazebo_msgs::ModelState msg;
    msg.model_name = "robot";
    msg.pose.position.x = 2.0;
    pub.publish(msg);

    sleep(1.0);
    ros::spinOnce();
    */
    return 0;
}