#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>


class Pattern {
private:
    double static constant_speed;
    double static rk;


public:
    static void circlePattern(geometry_msgs::Twist &m){

        m.linear.x = 2.0;
        m.linear.y = 0;
        m.linear.z = 0;

        m.angular.x = 0;
        m.angular.y = 0;
        m.angular.z = 1.8;
    }

    static void spiralPattern(geometry_msgs::Twist &m){
        rk += 0.5;

        m.linear.x = rk;
        m.linear.y = 0;
        m.linear.z = 0;

        m.angular.x = 0;
        m.angular.y = 0;
        m.angular.z = constant_speed;

    }

    static void randomPattern(geometry_msgs::Twist &m){
        m.linear.x =(double)(rand() % 20);
        m.linear.y =0;
        m.linear.z =0;

        m.angular.x = 0;
        m.angular.y = 0;
        m.angular.z =(double)(rand() % 10);

    }
};

double Pattern::constant_speed = 4.0;
double Pattern::rk = 0.5;

int main(int argc, char** argv) {

   ros::init(argc, argv, "lab1");
   ros::NodeHandle n;
   ros::Publisher vel_pub;


   vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
   ros::Rate loop(10);


   geometry_msgs::Twist msg;
   std::string type = "random";

   std::cout << "Please, choose your pattern (circle, spiral, random)\n";
   std::cin >> type;


   while (ros::ok) {

       if(type == "circle"){

           Pattern::circlePattern(msg);

       } else if(type == "spiral"){

           Pattern::spiralPattern(msg);

       } else if(type == "random"){

           Pattern::randomPattern(msg);

       }


       vel_pub.publish(msg);

       ros::spinOnce();
       loop.sleep();
   }

   return 0;
}


