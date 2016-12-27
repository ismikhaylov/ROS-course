 #include <ros/ros.h>
 #include <time.h>
 #include <math.h>
 #include <std_msgs/String.h>	
 #include <geometry_msgs/Twist.h>
 #include <geometry_msgs/Pose2D.h>
 #include <tf/transform_broadcaster.h>
 #include <tf/transform_listener.h>
 
 const int min = -7;
 const int max = 7;
 const double Pi = 3.14159;
 const float min_diff_x = 0.6;
 const float min_diff_y = 0.6;
 const float min_diff_angle = 1.5;
 float run_speed = 0.5;
 float rotate_speed = 0.6;
 bool isFound = false;
 bool isRotate = false;
 bool isRun = false;
 
 void callBack(const std_msgs::String::ConstPtr& msg) {
	isFound = true;
	isRun = false;
	isRotate = false;
	run_speed = 1.8;
	std::cout << "Robot1: follow Robot2" << std::endl;
 }

 int main(int argc, char **argv) {
	srand (time(NULL));
 	ros::init(argc,argv,"lost_robot");
 	ros::NodeHandle nh;
 	ros::Publisher velocity_publisher = nh.advertise<geometry_msgs::Twist>("/robot1/cmd_vel", 10);
 	ros::Subscriber sub = nh.subscribe("/robot_message/message", 10, callBack);
	ros::Rate r(30);
	
	geometry_msgs::Pose2D next_pos;
	next_pos.x = min + rand() % (max - min);
	next_pos.y = min + rand() % (max - min);
	std::cout << "new pose = " << next_pos.x << " " << next_pos.y << std::endl; 
	
	tf::TransformListener transformListener;
	tf::StampedTransform stf_self;
	tf::StampedTransform stf_robot2;

	while(ros::ok()){
		geometry_msgs::Twist velocity;
		try {
			if (!isFound) {
				transformListener.lookupTransform("robot1_tf/odom", "robot1_tf/footprint", ros::Time(0), stf_self);
			} else {
				transformListener.lookupTransform("robot1_tf/odom", "robot1_tf/footprint", ros::Time(0), stf_self);
				transformListener.lookupTransform("robot2_tf/odom", "robot2_tf/footprint", ros::Time(0), stf_robot2);
			}
		}
		catch (tf::TransformException &exeption) {
			ros::Duration(1.0).sleep();
			continue;
		}

		if (isFound) {
			next_pos.x = stf_robot2.getOrigin().x();
			next_pos.y = stf_robot2.getOrigin().y();
		}

		double angle;
		if (stf_self.getRotation().z() < 0) {
			angle = Pi*2 - stf_self.getRotation().getAngle();
		} else {
			angle = stf_self.getRotation().getAngle();
		}
		double grad_angle = fmod(angle*180/Pi + 180, 360);
		
		double rotate_angle = atan2(stf_self.getOrigin().y() - next_pos.y, stf_self.getOrigin().x() - next_pos.x)*180/Pi;
		if (rotate_angle < - 0.5) {
			rotate_angle = 360 + rotate_angle;
		}
		
		float diff_x = fabs(stf_self.getOrigin().x() - next_pos.x);
		float diff_y = fabs(stf_self.getOrigin().y() - next_pos.y);
			
		if (diff_x < min_diff_x && diff_y < min_diff_y) {
			velocity.linear.x = 0;
			velocity.angular.z = 0;
			velocity_publisher.publish(velocity);
			isRun = false;
			isRotate = false;
			
			if (!isFound) {
				next_pos.x = min + rand() % (max - min);
				next_pos.y = min + rand() % (max - min);
				std::cout << "new pose = " << next_pos.x << " " << next_pos.y << std::endl; 
			} else {
				ros::Duration(3).sleep();
			}
		} else {
			if (fabs(rotate_angle - grad_angle) > min_diff_angle) {
				if (!isRotate) {
					velocity.linear.x = 0;
					if (grad_angle < rotate_angle) {
						velocity.angular.z = rotate_speed;
					} else {
						velocity.angular.z = (-1) * rotate_speed;
					}
					velocity_publisher.publish(velocity);
					isRotate = true;
					std::cout << "publish rotate" << std::endl;
				}
			} else {
				if (!isRun) {
					velocity.linear.x = run_speed;
					velocity.angular.z = 0;
					velocity_publisher.publish(velocity);
					isRun = true;
					std::cout << "publish run" << std::endl;
				}
			}
			if (!isFound && isRun) {
				ros::Duration(4).sleep();
				isRun = false;
				isRotate = false;
			}
		}
	    ros::spinOnce();
	    r.sleep();
  	}
}
