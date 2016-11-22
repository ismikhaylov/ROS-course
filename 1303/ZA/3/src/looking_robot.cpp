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
 const int startPos[4][2] = {{-5, -5}, {5, -5}, {5, 5}, {-5, 5}};
 const int countPos = 4;
 bool isFound = false;

 int main(int argc, char **argv) {
	srand (time(NULL));
 	ros::init(argc,argv,"looking_robot");
 	ros::NodeHandle nh;
 	ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("lookr_topic",10,true);
 	ros::Publisher pubLostRobot = nh.advertise<std_msgs::String>("lost_robot_chatter",10);
	ros::Rate r(30);
	
	visualization_msgs::Marker msg;
    msg.header.frame_id = "/robot_on_map";
    msg.header.stamp = ros::Time::now();
    msg.ns = "looking_robot";
    msg.action = visualization_msgs::Marker::ADD;
    
    int randPos = rand()%4;
    int startX = startPos[randPos][0];
    int startY = startPos[randPos][1];
    msg.pose.position.x = startX;
    msg.pose.position.y = startY;
    msg.pose.position.z = 0.5;
    
    msg.id = 0;
    msg.type = visualization_msgs::Marker::SPHERE;
    
    msg.scale.x = 0.5;
    msg.scale.y = 0.5;
    msg.scale.z = 0.5;
    
    msg.color.r = 0.0;
    msg.color.g = 1.0;
    msg.color.b = 0.0;
    msg.color.a = 1.0;
	    
	pub.publish(msg);
	
	tf::TransformBroadcaster transformBroadcast;
	tf::Transform transform;
	tf::TransformListener transformListener;
	tf::StampedTransform stamTransform;
	
	while(ros::ok()){
		
		if (isFound) {
			float diff_x = fabs(msg.pose.position.x - startX);
			float diff_y = fabs(msg.pose.position.y - startY);
				
			if (diff_x < min_diff_x && diff_y < min_diff_y) {
				std::cout << "Looking robot:return to start." << std::endl;
			} else {
				if (diff_x >= min_diff_x) {
					float dx = speed + different_speed;
					dx *= msg.pose.position.x < startX ? 1 : -1;
					msg.pose.position.x += dx;
				}
				
				if (diff_y >= min_diff_y) {
					float dy = speed + different_speed;
					dy *= msg.pose.position.y < startY ? 1 : -1;
					msg.pose.position.y += dy;
				}
			}
		} else {
			try {
				transformListener.lookupTransform("/world", "/lost_robot/pose", ros::Time(0), stamTransform);
			}
			catch (tf::TransformException &exeption) {
				ros::Duration(1.0).sleep();
				continue;
			}

			float diff_x = fabs(msg.pose.position.x - stamTransform.getOrigin().x());
			float diff_y = fabs(msg.pose.position.y - stamTransform.getOrigin().y());
				
			if (diff_x <= min_diff_x && diff_y <= min_diff_y) {
				isFound = true;
				std_msgs::String message;
				message.data = "follow";
				pubLostRobot.publish(message);
				ros::spinOnce();
			} else {
				if (diff_x >= min_diff_x) {
					float dx = speed + different_speed;
					dx *= msg.pose.position.x < stamTransform.getOrigin().x() ? 1 : -1;
					msg.pose.position.x += dx;
				}
				
				if (diff_y >= min_diff_y) {
					float dy = speed + different_speed;
					dy *= msg.pose.position.y < stamTransform.getOrigin().y() ? 1 : -1;
					msg.pose.position.y += dy;
				}
			}
			
		}
	    pub.publish(msg);
	    
	    transform.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
		tf::Quaternion quaternion;
		quaternion.setRPY(0, 0, 0);
		transform.setRotation(quaternion);
		transformBroadcast.sendTransform(tf::StampedTransform(transform, ros::Time(0), "/world", "/looking_robot/pose"));
	    
	    ros::spinOnce();
	    r.sleep();
  	}
}
