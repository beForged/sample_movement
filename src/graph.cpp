#include <cmath>
#include <utility>
#include <vector>
#include <map>
#ifndef graph_h
#define graph_h
#include "../include/sample_movement/graph.h"
#endif

//https://stackoverflow.com/questions/5493474/graph-implementation-c
namespace rrt{
    /*
    struct vertex;
    
    //this is an edge
    struct adjacent{
        float distance;
        rrt::vertex *point;
        adjacent(rrt::vertex *v, float d) : distance(d) , point(v) {}
    };

    //2d coordinate representation, needs to be expandable to more than one 
    //coordinate ideally
    struct coordinate{
        float x;
        float y;
        coordinate(float x, float y) : x(x), y(y) {}
    };

    //adjacency list representation of graph node
    struct vertex{
        //adjacency list, with vertex, cost pair
        std::vector<adjacent> adj;
        //name i am not sure this is needed
        rrt::coordinate coordinate;
        //constructor
        vertex(rrt::coordinate c) : coordinate(c){}   
    };


	class graph{
		public:
            std::vector<vertex*> work;
            void addvertex(rrt::coordinate *c);
            void addedge(rrt::vertex *from, rrt::vertex *to, float cost);
	};	
    */
	
    void graph::addvertex(rrt::coordinate *c){
        //look for duplicate?
        vertex *v;
        v = new vertex(*c);
        work.push_back(v);
        //this is good if we need to check return type (std::pair<iterator, bool>)
        //work.insert(std::pair<std::string, vertex*>(name, v));
    }

    

    void graph::addedge(rrt::vertex *from, rrt::vertex *to, float cost){
        //vertex * a = pos->first;
        rrt::adjacent a = adjacent(to, cost);
        from->adj.push_back(a);
    }

}
