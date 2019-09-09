#include <cmath>
#include <utility>
#include <vector>
#include <map>

//https://stackoverflow.com/questions/5493474/graph-implementation-c
namespace rrt{
    struct vertex;
    
    struct adjacent{
        float distance;
        rrt::vertex *point;
        adjacent(rrt::vertex *v, float d) : distance(d) , point(v) {}
    };
    struct vertex{
        /*didnt end up using this
         * vertex is declared as a int node pair
         * typedef std::pair<int, node *> vertex;
         */
        //adjacency list, with vertex, cost pair
        std::vector<adjacent> adj;
        //name i am not sure this is needed
        std::string name;
        //constructor
        vertex(std::string s) : name(s) {}   
    };


	class graph{
		public:
            std::map<std::string, vertex*> work;
            //map work; //this is the map declared in line above (maybe rename)
            void addvertex(const std::string&);
            void addedge(const std::string& from, const std::string& to, float cost);
	};	
	
    void graph::addvertex(const std::string& name){
        //look for duplicate?
        vertex *v;
        v = new vertex(name);
        work[name] = v;
        //work.insert(n.name, n); ??
    }

    void graph::addedge(const std::string& from, const std::string& to, float cost){
        rrt::vertex *v = work[to];
        //vertex * a = pos->first;
        rrt::adjacent a = adjacent(v, cost);
        work[from]->adj.push_back(a);
    }

}
