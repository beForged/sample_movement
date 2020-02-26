#ifndef graph_h
#define graph_h
#include <cmath>
#include <utility>
#include <vector>
#include <map>

namespace rrt{
    struct vertex;

    struct adjacent{
        float distance;
        rrt::vertex *point;
        adjacent(rrt::vertex *v, float d) : distance(d), point(v) {}
    };

    struct coordinate{
        float x;
        float y;
        coordinate(float x, float y) : x(x), y(y) {}
        bool coordinate::operator ==(const coordinate &other) const{
            if(x == other.x && y == other.y){
                return true;
            }
            return false;
        }
    };

    struct vertex{
        std::vector<adjacent> adj;
        float goal_cost;
        float lcm;
        rrt::coordinate coordinate;
        rrt::vertex parent;
        vertex(rrt::coordinate c) : coordinate(c){}
        bool vertex::operator ==(const vertex &other) const{
            return coordinate == other.coordinate;
        }
    };

        

    class graph{
        public:
            std::vector<vertex*> work;
            void addvertex(rrt::coordinate *c);
            void addedge(rrt::vertex *from, rrt::vertex *to, float cost);
            std::vector<vertex> find_KNN(rrt::vertex *start, int n);
            float distance(rrt::vertex *one, rrt::vertex *two);

    };
}

#endif
