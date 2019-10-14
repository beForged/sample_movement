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
    };

    struct vertex{
        std::vector<adjacent> adj;
        rrt::coordinate coordinate;
        vertex(rrt::coordinate c) : coordinate(c){}
    };

    class graph{
        public:
            std::vector<vertex*> work;
            void addvertex(rrt::coordinate *c);
            void addedge(rrt::vertex *from, rrt::vertex *to, float cost);
    };
}
