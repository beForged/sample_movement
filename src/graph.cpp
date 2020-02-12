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

    float cdistance(coordinate one, coordinate two){
        float x = two.x - one.x;
        float y = two.y - two.y;
        return sqrt(x*x + y*y);
    }
    //finds the L2 distance
    float distance(rrt::vertex *one, rrt::vertex *two){
        return cdistance(one->coordinate, two->coordinate);
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
                //TODO make sure it doesnt count duplicates
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
    //moves vertex move towards start
    rrt::vertex get_moved(rrt::vertex *start, rrt::vertex *move, float r){
        rrt::coordinate vec = rrt::coordinate(start->coordinate.x - move->coordinate.x, start->coordinate.y - move->coordinate.y);
        float norm = sqrt(vec.x*vec.x + vec.y*vec.y);
        //normalize
        vec = rrt::coordinate((vec.x*r) / norm, (vec.y*r) / norm);
        return rrt::vertex(vec);
    }

    void remove_vertex(rrt::vertex *rem){
        //https://stackoverflow.com/questions/3385229/c-erase-vector-element-by-value-rather-than-by-position
        work.erase(std::remove(work.begin(), work.end(), v) vec.end());
        /*
        //find and vector::erase()
        for(vertex* v : work){
            //assume unique? just compare the pointers for now
            if(&rem == &v){
            }
        }
        */


    }



}
