#ifndef rrt_h
#define rrt_h
/*ROS libraries*/
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>

/*for global path planner*/
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

//std libraries
#include <iostream>
#include <cmath>
#include <set>
#include <string>
#include <vector>
#include <utility>
#include <boost/random.hpp>

//local
#include "graph.h"

namespace rrt{
class RRTPlanner: public nav_core::BaseGlobalPlanner{
 public:
	//constructors
	RRTPlanner();

	RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap);
	
	//overidden classes from nav_core base global planner
	void initialize(std::string name, costmap_2d::Costmap2DROS* costmap);

	bool makePlan(const geometry_msgs::PoseStamped& start,
		      const geometry_msgs::PoseStamped& goal,
		      std::vector<geometry_msgs::PoseStamped>& plan);

    bool makePath(const geometry_msgs::PoseStamped& start,
            const geometry_msgs::PoseStamped& goal,
            std::vector<geometry_msgs::PoseStamped>& plan);

    std::vector<Geometry_msgs::PoseStamped> pose_convert(std::vector<vertex> plan);
    
    std::vector<rrt::vertex> graph_search();

    std::vector<rrt:vertex> costmap_changes();

    bool costmap_change();

	bool testPath(rrt::vertex *start, rrt::vertex *end);

    int shrinkingball(int n);

    rrt::coordinate* makeRandPoint();
	
 private:
	//variables in rrt.cpp
	ros::NodeHandle node_handle_;
	bool initialized_;
	base_local_planner::WorldModel* world_model_;
	costmap_2d::Costmap2DROS* costmap_ros_;
	costmap_2d::Costmap2D* costmap_;
	int current_iterations_;
    int iterations;
    int max_iterations;
    int delta;

    
	
 };
}
#endif
