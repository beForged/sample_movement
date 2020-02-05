//add the macros and the header file in the includes folder
//TODO REMEMBER TO UNCOMMENT THIS 
//#include <pluginlib/class_list_macros.h>
#include "../include/sample_movement/rrt.h"
//#include "../include/nanoflann.hpp"
#include "graph.cpp"

//this is for exporting the plugin for cmake
PLUGINLIB_EXPORT_CLASS(rrt::RRTPlanner, nav_core::BaseGlobalPlanner)

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
			
            //need to add costmap/obstacle map and obstacles here (at least at first
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
        //we call make path here to set plan and see if completed or not from return
		
		plan.clear();
		current_iterations_ = 0;
		ROS_INFO("makePlan succeeded");
		return true;		
		
	}

    bool RRTPlanner::makePath(const geometry_msgs::PoseStamped& start,
                             const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan){
        /* 
         * RRT impl:
         * get a random point and add it to graph
         * find closest vertex (use kd tree)
         * forward simulate to new rand point
         * if successful is 1 iteration - and check if reached goal
         * else unsucessful then see if we reached max iterations and end if we have 
         * (endless iteration)
         */
        //initialize the graphs
        rrt::graph V;
        //TODO add knn graph here (kd tree or otherwise)
        //rrt::coordinate u;
        
        //add start node to graph
        //going to assume 2d navigation, graph will need to be modified TODO for non 2d navigation
        V.addvertex(new rrt::coordinate(start.pose.position.x, start.pose.position.y));

        bool goalReached = false;
        delta = 10;

        //check if robot is at goal or exceeded max iterations
        while(!goalReached || iterations < max_iterations){
        //randomly sample config space
            rrt::coordinate *u = makeRandPoint();
            //V.addvertex(u);
            //get closest member of V to u'
            //move u towards v until it is R distance away

            //add u to the graph first
            //need to add to graph becuase of graph implementation, change this later pls
            V.addvertex(*u);
            //find nearest point on graph to it
            ////this is a vector of vertexes TODO fix
            rrt::vertex nearest = find_KNN(*u,1);
            //R = min(delta, shringkingball(n)) only gets moved to R distance away
            float R = min(delta, shrinkingball(n)); //TODO need implement shrinkingball and init delta
            //this moves u to be within R distance of the graph
            rrt::vertex *v = get_moved(nearest, u, R);
            V.remove_vertex(*u);
            V.addvertex(*v);
            //attach parent vertex, and also all near ones become edges (how near?)
            V.addedge(v, nearest); //add parent edge
            //TODO need to check that it is safe to attach edges
            //add KNN to radius R here and attach edges
            //should actually be within a distance requires other knn function
            std::vector<vertex> parents = find_KNN(*v, 5);
            //add safe edges
            
            //w <- extend(v,u,epsilon) ?? w is graph, add new random point 
            //w <- extend(v, u, epsilon) move from v at u by epsilon

            //get closest member of V to 

            //find radius with k neighbors around new 
            //get those k neighbots
            //get all reachable neighbors
            //get closest of neighbots
            //if distance not zero 

        }
    }

	//forward simulate the robot against the costmap
    //should check that paths are possible or obstructed
    //TODO need to add params, start and end of path
	bool RRTPlanner::testPath(){
        //this is the ros costmap, careful of overflow, might be more efficent to move to 2d bool array
        //should be a separate function?
        int ix = 0;//temp
        int iy = 0;//temp
        char cost = static_cast<int>(costmap_ -> getCost(ix, iy));
        if(cost > 155){
            return false;
        }
        //need to check rest of path at some interval epsilon probably

        return true;
		
	}
    

    //generate a point within the map
    /*
     * We need to generate random points to seed the graph to plan in
     * TODO
     * in order to make sure we can actually get to the end, we need to manually
     * add the goal every n iterations so that it actually is in the graph
     */
    rrt::coordinate* RRTPlanner::makeRandPoint(){
        std::random_device rd;
        std::mt19937 gen(rd());
        float width = costmap_ -> getSizeInMetersX();
        float height= costmap_ -> getSizeInMetersY();
        std::uniform_real_distribution<> x(-width, width);
        std::uniform_real_distribution<> y(-height, height);

        //new_point.first = x(gen);
        //new_point.second= y(gen);
        float xres = x(gen);
        float yres = y(gen);
        
        return new rrt::coordinate(xres, yres);
    }

    int shrinkingball(int n){
        //placeholder for now
        return 10;
    }

	
}
