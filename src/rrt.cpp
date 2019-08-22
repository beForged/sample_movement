//add the macros and the header file in the includes folder
#include <pluginlib/class_list_macros.h>
#include "../include/sample_movement/rrt.h"
#include "../include/nanoflann.hpp"

//this is for exporting the plugin for cmake
PLUGINLIB_EXPORT_CLASS(rrt::RRTPlanner, nav_core::BaseGlobalPlanner);

namespace rrt{
	//checking if costmap is null or init is false?
/*
	RRTPlanner::RRTPlanner()
	: costmap_(nullptr), intitialized_(false) { }
*/

	RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		if(!initialized_){
		//start map
			costmap_ros_ = costmap_ros;
			costmap_ = costmap_ros->getCostmap();	

			
			//new node handle
			ros::NodeHandle node("~/rrt");
			node_handle_ = node;
			world_model_ = new base_local_planner::CostmapModel(*costmap_);
			
			//get parameters (from .yaml file?)

			
//			node_handle_.getParam("/move_base/delta_", delta_);
//			node_handle_.getParam("/move_base/goal_radius_", goal_radius_);
//			node_handle_.getParam("/move_base/max_iterations_", max_iterations_);

//			ROS_INFO("step size: %.2f , goal radius: %.2f, delta: %2f, max iterations: %2f", 
//			step_size_, goal_radius_, delta_, max_iterations_);
			
			current_iterations_ = 0;
			
			ROS_INFO("planner initialized");
			initialized_ = true;
		}else{
			ROS_WARN("planner already initialized!");
		}
	}//end constructor

	
	bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start,
				  const geometry_msgs::PoseStamped& goal,
				  std::vector<geometry_msgs::PoseStamped>& plan){
		if(!initialized_){
			ROS_ERROR("not initialized yet, call initialize()");
			return false;
		}
		//this is temp placeholder stuff
		//TODO
		
		plan.clear();
		current_iterations_ = 0;
		ROS_INFO("makePlan succeeded");
		return true;		
		
	}

    int RRTPlanner::makePath(const geometry_msgs::PoseStamped& start,
                             const geometry_msgs::PoseStamped& goal){
        /* 
         * RRT impl:
         * get a random point and add it to graph
         * find closest vertex (use kd tree)
         * forward simulate to new rand point
         * if successful is 1 iteration - and check if reached goal
         * else unsucessful then see if we reached max iterations and end if we have 
         * (endless iteration)
         */
    }

	//forward simulate the robot against the costmap
	bool RRTPlanner::testPath(){
		
	}

    //generate a point within the map
    /*
     * We need to generate random points to seed the graph to plan in
     * TODO
     * in order to make sure we can actually get to the end, we need to manually
     * add the goal every n iterations so that it actually is in the graph
     */
    std::pair<float,float> RRTPlanner::makeRandPoint(){
        std::pair<float,float> new_point;
        std::random_device rd;
        std::mt19937 gen gen(rd());
        float width = costmap_ -> getSizeInMetersX();
        float height= costmap_ -> getSizeInMetersY();
        std::uniform_real_distribution<> x(-width, width);
        std::uniform_real_distribution<> y(-height, height);

        new_point.first = x(gen);
        new_point.second= y(gen);
        
        return new_point;
    }
	
}
