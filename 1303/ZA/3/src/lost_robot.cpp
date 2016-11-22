 #include <ros/ros.h>
 #include <visualization_msgs/Marker.h>  
 #include <turtlesim/Pose.h>
 #include <time.h>
 #include <math.h>
 #include <std_msgs/String.h>
 #include <tf/transform_broadcaster.h>
 #include <tf/transform_listener.h>
 
 const int min = -5;
 const int max = 6;
 const float min_diff_x = 0.2;
 const float min_diff_y = 0.2;
 const float speed = 0.05;
 const float different_speed = -0.01;
 bool isFound = false;
 
 void callBack(const std_msgs::String::ConstPtr& msg) {
	std::cout << "new message" << std::endl;
	isFound = msg->data == "follow";
 }

 int main(int argc, char **argv) {
	srand (time(NULL));
 	ros::init(argc,argv,"lost_robot");
 	ros::NodeHandle nh;
 	ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("lr_topic", 10 ,true);
 	ros::Subscriber sub = nh.subscribe("lost_robot_chatter", 10, callBack);
	ros::Rate r(30);
	
	visualization_msgs::Marker msg;
    msg.header.frame_id = "/robot_on_map";
    msg.header.stamp = ros::Time::now();
    msg.ns = "lost_robot";
    msg.action = visualization_msgs::Marker::ADD;
    
    msg.pose.position.x = min + rand() % (max - min);
    msg.pose.position.y = min + rand() % (max - min);
    msg.pose.position.z = 0.5;
    
    msg.id = 0;
    msg.type = visualization_msgs::Marker::SPHERE;
    
    msg.scale.x = 0.5;
    msg.scale.y = 0.5;
    msg.scale.z = 0.5;
    
    msg.color.r = 0.0;
    msg.color.g = 0.0;
    msg.color.b = 1.0;
    msg.color.a = 1.0;
	    
	pub.publish(msg);
	
	turtlesim::Pose next_pos;
	next_pos.x = min + rand() % (max - min);
	next_pos.y = min + rand() % (max - min);
	
	tf::TransformBroadcaster transformBroadcast;
	tf::Transform transform;
	tf::TransformListener transformListener;
	tf::StampedTransform stamTransform;
	
	while(ros::ok()){
		
		if (!isFound) {
			float diff_x = fabs(msg.pose.position.x - next_pos.x);
			float diff_y = fabs(msg.pose.position.y - next_pos.y);
				
			if (diff_x < min_diff_x && diff_y < min_diff_y) {
				next_pos.x = min + rand() % (max - min);
				next_pos.y = min + rand() % (max - min);
			} else {
				if (diff_x >= min_diff_x) {
					float dx = speed;
					dx *= msg.pose.position.x < next_pos.x ? 1 : -1;
					msg.pose.position.x += dx;
				}
				
				if (diff_y >= min_diff_y) {
					float dy = speed;
					dy *= msg.pose.position.y < next_pos.y ? 1 : -1;
					msg.pose.position.y += dy;
				}
			}
		} else {
			try {
				transformListener.lookupTransform("/world", "/looking_robot/pose", ros::Time(0), stamTransform);
			}
			catch (tf::TransformException &exeption) {
				ros::Duration(1.0).sleep();
				continue;
			}

			float diff_x = fabs(msg.pose.position.x - stamTransform.getOrigin().x());
			float diff_y = fabs(msg.pose.position.y - stamTransform.getOrigin().y());
				
			if (diff_x >= min_diff_x) {
				float dx = speed + different_speed;
				dx *= msg.pose.position.x < stamTransform.getOrigin().x() ? 1 : -1;
				msg.pose.position.x += dx;
			}
			
			if (diff_y >= min_diff_y) {
				float dy = speed +different_speed;
				dy *= msg.pose.position.y < stamTransform.getOrigin().y() ? 1 : -1;
				msg.pose.position.y += dy;
			}
		}
	    pub.publish(msg);
	    
	    transform.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
		tf::Quaternion quaternion;
		quaternion.setRPY(0, 0, 0);
		transform.setRotation(quaternion);
		transformBroadcast.sendTransform(tf::StampedTransform(transform, ros::Time(0), "/world", "/lost_robot/pose"));
	    
	    ros::spinOnce();
	    r.sleep();
  	}
}
