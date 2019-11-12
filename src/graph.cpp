#include <cmath>
#include <utility>
#include <vector>
#include <map>
#include <math.h>
#include <algorithm>
#include "../include/sample_movement/graph.h"

//https://stackoverflow.com/questions/5493474/graph-implementation-c
namespace rrt{

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

    //finds the L2 distance
    float distance(rrt::vertex *one, rrt::vertex *two){
        float x = two->coordinate.x - one->coordinate.x;
        float y = two->coordinate.y - two->coordinate.y;
        return sqrt(x*x + y*y);
    }

    //finds K nearest neighbors. 
    //This is a raw brute force O(dS) just for MVP purposes
    //*start is node to find neighbors of 
    //n is number of neighbors to find
    std::vector<vertex> graph::find_KNN(rrt::vertex *start, int k){
        int least = 0;
        std::vector<vertex*> lst [k];
        for(int i = 0; i < k; i++){ 
            for(vertex* point : work){       
                float d = rrt::distance(start,point);
                bool contains = true;
                for(int j = 0; j < k; j++){
                    if(lst[j] == point){
                        contains = false;
                    }
                }
                if(d < least && contains){
                    lst[i] = point;
                    least = d;
                }
            }
            
        }
        return lst;
    }

    //for L2 distance, gets point at which
    rrt::vertex get_moved(rrt::vertex *start, rrt::vertex *move, float r){
        rrt::coordinate vec = rrt::coordinate(start->coordinate.x - move->coordinate.x, start->coordinate.y - move->coordinate.y);
        float norm = sqrt(vec.x*vec.x + vec.y*vec.y);
        //normalize
        vec = rrt::coordinate((vec.x*r) / norm, (vec.y*r) / norm);
        return rrt::vertex(vec);



    }



}
