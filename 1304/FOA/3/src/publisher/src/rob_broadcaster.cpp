#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

//----

void poseCallback(const visualization_msgs::MarkerConstPtr& msg){
 static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->pose.orientation.z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "r1"));
ROS_INFO("Pose:%f %f %f",msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "broadcaster1");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("visualization_marker1", 1000, &poseCallback);
  ros::spin();
  return 0;
}