#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <stdio.h>


void callback(const sensor_msgs::LaserScan::ConstPtr& scan){
//	ROS_INFO("Hello\n");
//	ROS_INFO("ranges = [%f]\n", scan->ranges[0]);
//	int count = scan->scan_time / scan->time_increment;
//	for(int i = 0; i < count; i++){
//		ROS_INFO("ranges = [%f]\n", scan->ranges[i]);
//	} 
}

void printMap(const nav_msgs::OccupancyGrid::ConstPtr& map){
	int s = sizeof(map->data);
	ROS_INFO("mapsize: %d", s);
	//const std::vector<char> m = map->data;
/*
	for(int i = 0; i < sizeof(m) / sizeof(int); i++){
		ROS_INFO("%d", m[i]);
	}
*/
}

int main(int argc, char **argv){
	ros::init(argc, argv, "listen");
	ros::NodeHandle nh;

	//ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, callback);
	ros::Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 10, printMap);
	ROS_INFO_STREAM("init sub");

	ros::spin();
	return 1;
}
