#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include "octomap_listener/point.hh"
#include "octomap_listener/vertex.hh"

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>
#include <memory>
#include <mutex>

class Graph {

typedef std::unordered_map<std::string, std::shared_ptr<Vertex>> ADJ;
typedef std::shared_ptr<Vertex> VertexPtr;

public:
    Graph();
    Graph(double, double, double, int);
    void addVertex(const Vertex&);
    void addToNearestVertex(Vertex&, bool);
    Vertex getNearestVertexInGraph(const Vertex&);
    void removeVerticesInHole();
    void removeVertex(Vertex&);
    void addEdge(const Vertex&, const Vertex&);
    void addEdge(const std::string&, const std::string&);
    void addEdge(ADJ::iterator&, const std::string&);
    void addEdge(ADJ::local_iterator&, const std::string&);
    void printAdjList() const;
    void saveAdjList() const;
    void makeGridMap(double, double, double, double);
    void constructEdges();
    void process_neighbors(const std::vector<Vertex>&, ADJ::iterator&, int);
    double getGroundHeight() const;
    bool isNearHole(const Vertex&) const;
    Vertex getNearestDiamond(const Vertex&, bool&);

    //const Vertex* findPath(const Point&, const Point&);
    VertexPtr findPath(const Vertex&, const Vertex&);

    std::unordered_map<std::string, std::shared_ptr<Vertex>> adjacency_map_;
    std::unordered_map<std::string, std::shared_ptr<Vertex>> hole_map_;
    std::unordered_map<std::string, Vertex> diamond_vect_;
    std::unordered_set<std::string> visited_diamonds_;
private:
    double coeff_res_;
    double vertex_res_;
    double ground_height_;
    double threshold_height_;
    double hole_neighbor_depth_;
    int diamond_size_;
    std::mutex graph_mutex_;

    void clearVertexPredecessors();
    bool existsEdge(const Vertex&, const std::string&) const;
    void constructEdgesParallel(int);
    double get_distance(const Point&, const Point&) const;
    Point getCentralizedPoint(Point);
    void setToNearestPoint(Vertex&);
    void setToNearestDiamond(Vertex&);
    void getNeighborList(std::vector<Vertex>&, Point, int depth = 1);
};


#endif  // GRAPH_HPP_
