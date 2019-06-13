#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>

int main(int argc, char **argv){

	ROS_INFO_STREAM("hewwo");

	ros::init(argc, argv, "move");
	ros::NodeHandle nh;

	//publisher
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	ROS_INFO_STREAM("init publisher");

	//init direction
	geometry_msgs::Twist base_cmd;
	geometry_msgs::Twist base_cmd_turn_left;

	base_cmd.linear.x = 0;  
	base_cmd.linear.y = 0;  
	base_cmd.angular.z = 0;
	base_cmd_turn_left.linear.x = 0;  
	base_cmd_turn_left.linear.y = 0;  
	base_cmd_turn_left.angular.z = 0;

	//move forward
	base_cmd.linear.x = 0.25;
	base_cmd.angular.z = 0.0;
	ROS_INFO_STREAM("ctrl + c to stop");


	base_cmd_turn_left.linear.x = 0;
	base_cmd_turn_left.angular.z = 1.57/2;
	
	ros::Rate rate(5);

	ROS_INFO_STREAM("start movement");

	for(int i = 0; i < 2; i++){
		for(int n = 10; n > 0; n--){
			pub.publish(base_cmd);
			rate.sleep();
		}

		for(int n = 10; n > 0; n--){
			pub.publish(base_cmd_turn_left);
			rate.sleep();
		}

	}
	//listener for speed i guess
	

	ROS_INFO_STREAM("end movement");

	base_cmd.linear.x = 0;
	base_cmd.linear.y = 0;
	base_cmd.angular.z = 0;
	pub.publish(base_cmd);
	rate.sleep();
	
	ROS_INFO("finished \n");

	return 0;

}

