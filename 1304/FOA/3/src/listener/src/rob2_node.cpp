#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
void waiting(ros::Publisher pub,int time){
    while (pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return ;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(time);
    }
    
}

void moveRobot(const geometry_msgs::TwistConstPtr &transform){
    double tX=transform->linear.x;
    double tY=transform->linear.y;
    double tZ=transform->linear.z;
    ROS_INFO("Transform %f %f %f",tX,tY,tZ);
}

void drawRobot(double startX,double startY,double startZ){
    ros::NodeHandle nh;
    ros::Publisher pub =
            nh.advertise<visualization_msgs::Marker>("visualization_marker2",1);
    ros::Rate r(30);
    uint32_t shape = visualization_msgs::Marker::CUBE;

    double endX;
    double endY;
    double endZ=0.5;

    double offsetX = 1;
    double offsetY = 1;
    visualization_msgs::Marker msg;
    msg.header.frame_id = "/my_frame";
    msg.header.stamp = ros::Time::now();
    msg.ns = "robot2";
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.position.x = startX;
    msg.pose.position.y =startY;
    msg.pose.position.z = startZ;
    
    msg.id = 0;
    msg.type = shape;
    
    msg.scale.x = 1.0;
    msg.scale.y = 1.0;
    msg.scale.z = 1.0;
    
    msg.color.r = 0.0f;
    msg.color.g = 1.0f;
    msg.color.b = 0.0f;
    msg.color.a = 5.0;
    
    ros::NodeHandle node;
    tf::TransformListener listener;
    bool isFind=false;
    double L2=0;
    while(ros::ok())
    {
        waiting(pub,1);

        sleep(2);
        tf::StampedTransform transform;
        try{

            listener.lookupTransform("r2", "r1",
                                     ros::Time(0), transform);
            //sleep(2);
            ROS_INFO("Transform %f %f %f", transform.getOrigin().getX(),transform.getOrigin().getY(),transform.getOrigin().getZ());
            double tX= transform.getOrigin().getX();
            double tY=transform.getOrigin().getY();
            double tZ=transform.getOrigin().getZ();
            double x1,y1,x2,y2;
            x1=msg.pose.position.x;
            y1=msg.pose.position.y;
            x2=x1+tX;
            y2=y1+tY;
            if(msg.pose.position.x<=tX+1&&msg.pose.position.y<=tY+1)
            {

                ROS_INFO("Pos: %f %f",msg.pose.position.x,msg.pose.position.y);
                msg.pose.position.x +=offsetX;
                msg.pose.position.y +=offsetY;
                msg.pose.position.z = 0.5;

            }
            else
            {
                isFind=true;
                endX=msg.pose.position.x;
                endY=msg.pose.position.y;
            }

            if(isFind){
                ROS_INFO("isFind:%d",isFind);
                if(msg.pose.position.x>=startX&& msg.pose.position.y>=startY)
                {
                    msg.pose.position.x = (endX-=offsetX);
                    msg.pose.position.y = (endY-=offsetY);
                    msg.pose.position.z = 0.5;
                }else{
                    break;
                    ROS_INFO("BAD MOVE:%f %f",msg.pose.position.x, msg.pose.position.y);
                }

            }

            pub.publish(msg);
        }
        catch (tf::TransformException &ex) {
            pub.publish(msg);
            ROS_ERROR("%s",ex.what());
            ros::Duration(2.0).sleep();
            continue;
        }

    }
    r.sleep();
}

int main(int argc, char **argv) {
    ros::init(argc,argv,"node2");
    drawRobot(-5,-5,0.5);
    sleep(2);
    return 0;
}
