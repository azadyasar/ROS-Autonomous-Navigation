#include <octomap_listener/graph.hh>
#include <iostream>
#include <cmath>
#include <queue>
#include <octomap_listener/fpqueue.hh>
#include <octomap/octomap.h>

#include <ros/ros.h>

#include <time.h>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <istream>
#include <thread>


Graph::Graph() : vertex_res_(1) {
    std::cout << "[Graph:Graph]: default res_ = " << vertex_res_ << std::endl;
}

Graph::Graph(double _res, double _ground_height, double _threshold_height, int _diamond_size)
 : vertex_res_(_res), ground_height_(_ground_height), 
 threshold_height_(_threshold_height) {
    coeff_res_ = 1. / vertex_res_;
    hole_neighbor_depth_ = floor(coeff_res_ / 2);
    this->diamond_size_ = _diamond_size;
    std::cout << "Coeff: " << coeff_res_ 
    << ", depth: " << hole_neighbor_depth_ << std::endl;
 }

void Graph::addVertex(const Vertex &_vertex) {
    ADJ::iterator it = this->adjacency_map_.find(_vertex.get_name());
    if (it == this->adjacency_map_.end()) {
        VertexPtr tmp = std::make_shared<Vertex>(_vertex);
        this->adjacency_map_.insert(std::make_pair(tmp->get_name(), tmp));
    }//else
       // std::cout << "[addVX]: " <<  _vertex.get_name() << " already exists" << std::endl;
}

/* no need to check occupanc for second if the cell is not occupied
    the height will be 0.4 */
void Graph::addToNearestVertex(Vertex& _vertex, bool _is_occupied) {
    //Point nearest_point_tmp = getNearestPoint(_vertex.get_center_point());
   // std::cout << "Nearest point of " << _vertex << " ";
    setToNearestPoint(_vertex);
    Vertex point_diamond = _vertex;
    setToNearestDiamond(point_diamond);
    std::unordered_map<std::string, Vertex>::iterator p_it = this->diamond_vect_.find(point_diamond.get_name());
    if (p_it == this->diamond_vect_.end() && 
        this->visited_diamonds_.find(point_diamond.get_name()) == this->visited_diamonds_.end()) 
    {
        this->diamond_vect_.insert(std::make_pair(point_diamond.get_name(),
            point_diamond));
        // std::cout << "Inserted " << point_diamond << std::endl;
    }
   // std::cout << _vertex << std::endl;
    ADJ::iterator hole_it = this->hole_map_.find(_vertex.get_name());
    if (hole_it != this->hole_map_.end()) {
        return; 
    }
    ADJ::iterator it = this->adjacency_map_.find(Vertex(_vertex).get_name());
    double height_tmp = _vertex.get_height();
    if (it == this->adjacency_map_.end()) {
        VertexPtr new_vx_ptr = std::make_shared<Vertex>(_vertex); // new Vertex(_vertex);
        //new_vx_ptr->set_center_point(_vertex.get_center_point());
        new_vx_ptr.get()->cell_size_++;
        this->adjacency_map_.insert(std::make_pair(new_vx_ptr->get_name(), new_vx_ptr));
        //std::cout << "[Graph::addToNearestVertex]: Added " << new_vx_ptr << std::endl;
    } else if (it->second.get()->get_height() < height_tmp && it->second.get()->get_height() >= ground_height_) {
       // std::cout << "[1Graph::addToNearestVertex]: Updated height of " << 
        //    it->second.get() << " to " << _vertex.get_height() << std::endl;
        it->second.get()->set_height(height_tmp);
        it->second.get()->cell_size_++;
    } else if (_is_occupied && height_tmp < ground_height_ 
        && height_tmp < it->second.get()->get_height()) 
    {
        //std::cout << "[2Graph::addToNearestVertex]: Updated height of " << 
         //   it->second.get() << " to " << _vertex.get_height() << std::endl;
        it->second.get()->set_height(height_tmp);
        it->second.get()->cell_size_++;
    } else
        it->second.get()->cell_size_++;
}

Vertex Graph::getNearestVertexInGraph(const Vertex& _vertex){
    ADJ::iterator it = this->adjacency_map_.begin();
    double min_distance = get_distance(_vertex.get_center_point(), 
        it->second.get()->get_center_point());
    Vertex nearest_vertex = *it->second.get();
    for (ADJ::iterator end = this->adjacency_map_.end(); it != end; it++) {
        double current_diff = get_distance(_vertex.get_center_point(),
            it->second.get()->get_center_point());
        if (current_diff < min_distance) {
            min_distance = current_diff;
            nearest_vertex = *it->second.get();
        }
    }
    return nearest_vertex;
}

// Now removes the walls as well
void Graph::removeVerticesInHole() {
    std::cout << "~~~~~~LOCKING [remove]\n";
    std::lock_guard<std::mutex> lock(this->graph_mutex_);
    std::cout << "~~~~~~LOCKED [remove]\n";
    for (ADJ::iterator it = this->adjacency_map_.begin(), end = this->adjacency_map_.end();
        it != end;)
    {
        // std::cout << *it->second.get() <<" - Cell size: " << it->second.get()->cell_size_ << std::endl;
        // it->second.get()->cell_size_ = 0;
        VertexPtr hole_vx;
        if ( it->second.get()->get_height() < ground_height_) {
            std::vector<Vertex> neighbor_list;
            getNeighborList(neighbor_list, it->second.get()->get_center_point(), hole_neighbor_depth_);
            for (auto &it_vect : neighbor_list) {
                ADJ::iterator it_neighbor = this->adjacency_map_.find(it_vect.get_name());
                if (it_neighbor != this->adjacency_map_.end()) {
                   // std::cout << "Removing hole neighbor " << it_neighbor->second.get() << std::endl;
                    hole_vx = it_neighbor->second;
                    this->hole_map_.insert(std::make_pair(hole_vx.get()->get_name(), hole_vx));
                    it_neighbor->second.reset();
                    this->adjacency_map_.erase(it_neighbor);
                }
            }
           // std::cout << "Removing " << it->second.get() << std::endl;
            hole_vx = it->second;
            this->hole_map_.insert(std::make_pair(hole_vx.get()->get_name(), hole_vx));
            it->second.reset();
            it = this->adjacency_map_.erase(it);

        } else if (it->second.get()->get_height() > 0.6) {
            std::vector<Vertex> neighbor_list;
            getNeighborList(neighbor_list, it->second.get()->get_center_point(), hole_neighbor_depth_+1);
            for (auto &it_vect : neighbor_list) {
                ADJ::iterator it_neighbor = this->adjacency_map_.find(it_vect.get_name());
                if (it_neighbor != this->adjacency_map_.end()) {
                    // std::cout << "Removing wall neighbor " << it_neighbor->second.get() << std::endl;
                    it_neighbor->second.reset();
                    this->adjacency_map_.erase(it_neighbor);
                }
            }
            // std::cout << "Removing wall" << it->second.get() << std::endl;
            it->second.reset();
            it = this->adjacency_map_.erase(it);
        }else
            it++;
    }
}

void Graph::removeVertex(Vertex& _vertex) {
    std::lock_guard<std::mutex> lock(this->graph_mutex_);
    setToNearestPoint(_vertex);
    ADJ::iterator vx_it, hole_it;
    std::vector<Vertex> neighbor_list;
    getNeighborList(neighbor_list, _vertex.get_center_point());
    for (auto &it_vect : neighbor_list) {
        vx_it = this->adjacency_map_.find(it_vect.get_name());
        hole_it = this->hole_map_.find(it_vect.get_name());
        if (vx_it != this->adjacency_map_.end()) {
            // std::cout << "--Removing from adj\n";
            vx_it->second.reset();
            this->adjacency_map_.erase(vx_it);
        }
        if (hole_it == this->hole_map_.end()) {
            // std::cout << "Adding hole\n";
            this->hole_map_.insert(std::make_pair(it_vect.get_name(), 
                    std::make_shared<Vertex>(it_vect)));
        }
    }
    vx_it = this->adjacency_map_.find(_vertex.get_name());
    hole_it = this->hole_map_.find(_vertex.get_name());
    if (vx_it != this->adjacency_map_.end()) {
            // std::cout << "--Removing from adj\nx\nx\nx\nx\nx\n";
            vx_it->second.reset();
            this->adjacency_map_.erase(vx_it);
    } else {
        // std::cout << "--Not Removing from adj\nx\nx\nx\nx\nx\n";
    }
    if (hole_it == this->hole_map_.end()) {
        // std::cout << "Adding hole\nx\nx\nx\nx\nx\n";
        this->hole_map_.insert(std::make_pair(_vertex.get_name(), 
                std::make_shared<Vertex>(_vertex)));
    } else {
        // std::cout << "Not Removing from adj\nx\nx\nx\nx\nx\n";
    }
}


//  TOCHECK should the vertices be added or skipped?
void Graph::addEdge(const Vertex& _from, const Vertex& _to) {
    ADJ::iterator it_from = this->adjacency_map_.find(_from.get_name());
    if (it_from == this->adjacency_map_.end()){
        return;
    }
    ADJ::iterator it_to = this->adjacency_map_.find(_to.get_name());
    if (it_to == this->adjacency_map_.end()) {
        return;
    }
    it_from = this->adjacency_map_.find(_from.get_name());
    it_from->second.get()->outgoing_links_.push_back(std::make_shared<Vertex>(_to));
}

void Graph::addEdge(const std::string &_from, const std::string &_to) {
    ADJ::iterator it_from = this->adjacency_map_.find(_from);
    if (it_from == this->adjacency_map_.end()){
        std::cout << "[addEdge]: " << _from << " does not exist. Returning.." << std::endl;
        return;
    }
    ADJ::iterator it_to = this->adjacency_map_.find(_to);
    if (it_to == this->adjacency_map_.end()) {
        std::cout << "[addEdge]: " << _to << " does not exist. Returning.." << std::endl;
        return;
    }
    it_from = this->adjacency_map_.find(_from);
    it_from->second.get()->outgoing_links_.push_back(it_to->second);
}

void Graph::addEdge(ADJ::iterator& _it_from, const std::string& _to) {
    ADJ::iterator it_to = this->adjacency_map_.find(_to);
    if (it_to == this->adjacency_map_.end())
        return;
    /*std::cout << "Adding edge from " << _it_from->second->get_name() <<
        " to " << _to << std::endl; */
    _it_from->second.get()->outgoing_links_.push_back(it_to->second);
}

void Graph::addEdge(ADJ::local_iterator& _it_from, const std::string& _to) {
    std::lock_guard<std::mutex> lock_graph(graph_mutex_);
    ADJ::iterator it_to = this->adjacency_map_.find(_to);
    if (it_to == this->adjacency_map_.end())
        return;
    /*std::cout << "Adding edge from " << _it_from->second->get_name() <<
        " to " << _to << std::endl; */
    _it_from->second.get()->outgoing_links_.push_back(it_to->second);
}

void Graph::printAdjList() const {
    for (ADJ::const_iterator it = adjacency_map_.begin(), end = adjacency_map_.end(); it != end; it++) {
        std::cout << "At " << it->second << "--> ";
        const Vertex& tmp = *it->second.get();
        for (std::list<VertexPtr>::const_iterator it = tmp.outgoing_links_.begin(), end = tmp.outgoing_links_.end(); it != end; it++)
            std::cout << (*it).get()->get_name() << ", ";
        std::cout << std::endl;
    }
}

void Graph::saveAdjList() const {
    std::ofstream fout;
    srand(time(NULL));
    int random = rand();
    std::string file_base_name = std::string("/home/azad/ros_workspace/octomap_ws/data/adj_list_info_");
    std::string file_name = file_base_name +
        std::to_string(random) + std::string(".txt");
    fout.open(file_name);
    std::stringstream ss("Info: ", std::ios_base::app | std::ios_base::out);
    for (ADJ::const_iterator it = adjacency_map_.begin(), end = adjacency_map_.end();
        it != end; it++) {
        ss << "At " << it->second << "-->";
        const Vertex& tmp = *it->second.get();
        for (std::list<VertexPtr>::const_iterator it = tmp.outgoing_links_.begin(), end = tmp.outgoing_links_.end();
        it != end; it++) {
            ss << (*it).get()->get_name() << ", ";
        }
        ss << std::endl;
    }
    fout << ss.str();
}

void Graph::makeGridMap(double _xmin, double _xmax, double _ymin, double _ymax) {
    srand(time(NULL));
    int vertex_counter = 0;
    for (double i_x = _xmin; i_x <= _xmax; i_x+=vertex_res_) {
        for (double i_y = _ymin; i_y <= _ymax; i_y+=vertex_res_) {
            Point central_point(i_x, i_y); //getCentralizedPoint(Point(i_x, i_y));
            Vertex v_tmp(central_point);
            v_tmp.set_height((rand() % 20) / 5.);
            std::cout << "[INFO]: Made vertex #" << ++vertex_counter << ": " << central_point << " H: " << v_tmp.get_height() << std::endl;
            addVertex(v_tmp);
        }
    }
    constructEdges();
}
/*
Not used currently
void Graph::process_neighbors(const std::vector<Vertex>& _neighbor_list, ADJ::iterator& _it_from, int _part) {

    switch (_part) {
        case 0:
            for (int i = 0, end = _neighbor_list.size()/2; i < end; i++) {
                Vertex vx_tmp = _neighbor_list[i];
                ADJ::iterator it_tmp = adjacency_map_.find(vx_tmp.get_name());
                if (it_tmp != adjacency_map_.end()) {
                    double diff = it_tmp->second->get_height() - 
                        _it_from->second->get_height();
                    if ( diff <= threshold_height_) {
                        addEdge(_it_from, it_tmp->second->get_name());
                    }
                }
            }
            break;
        case 1:
            for (int i = _neighbor_list.size()/2, end = _neighbor_list.size(); i < end; i++) {
                Vertex vx_tmp = _neighbor_list[i];
                ADJ::iterator it_tmp = adjacency_map_.find(vx_tmp.get_name());
                if (it_tmp != adjacency_map_.end()) {
                    double diff = it_tmp->second->get_height() - 
                        _it_from->second->get_height();
                    if ( diff <= threshold_height_) {
                        addEdge(_it_from, it_tmp->second->get_name());
                    }
                }
            }
            break;
    }

}
*/

// TOCHECK robot is free to fall from above!!
void Graph::constructEdges() {
    std::thread t0(&Graph::constructEdgesParallel, this, 0);
    std::thread t1(&Graph::constructEdgesParallel, this, 1);
    std::thread t2(&Graph::constructEdgesParallel, this, 2);
    std::thread t3(&Graph::constructEdgesParallel, this, 3);
    t0.join();
    t1.join();
    t2.join();
    t3.join();
    /*
    for (ADJ::iterator it = adjacency_map_.begin(), end = adjacency_map_.end(); it != end && ros::ok(); it++) {
        std::vector<Vertex> neighbor_list;
        getNeighborList(neighbor_list, it->second.get()->get_center_point());
        counter++;/*
        std::thread t1(&Graph::process_neighbors, this, 
            std::ref(neighbor_list), std::ref(it), 0);
        std::thread t2(&Graph::process_neighbors, this, 
            std::ref(neighbor_list), std::ref(it), 1);
        t1.join();
        t2.join();
        double total_time = 0.0;
        for (auto &it_inner : neighbor_list) {
            // Vertex is read from adj_map because when #getNeighborList func pushes neighbor vertexes it does not know about height attribute of
             //   the vertex 
            ADJ::iterator it_tmp = adjacency_map_.find(it_inner.get_name());
            
            if (it_tmp != adjacency_map_.end()) {
                double diff = it_tmp->second->get_height() - it->second->get_height();
                if ( diff <= threshold_height_ ) {
                    clock_t start = clock();
                    addEdge(it, it_inner.get_name());
                    total_time += (double)(clock() - start) / CLOCKS_PER_SEC;
                }
            } 
        } 
        std::cout << "Took "<< total_time << " secs" << std::endl; 
        if (static_cast<int>(counter) % 50 == 0) {
            std::cout.precision(8);
            batch_counter += 50;  
            std::cout << "#" << (int)counter <<"Processed " << 
            batch_counter << "/" << size << std::endl;
        } 
    }*/
}

// CHECK is addEdge function risky in concurrent mode? 
void Graph::constructEdgesParallel(int _part) {
    switch (_part) {
        case 0:
            for (int i = 0, bucket_end = adjacency_map_.bucket_count()/4;
                i < bucket_end; i++)
            {
                for (ADJ::local_iterator it = adjacency_map_.begin(i), end = adjacency_map_.end(i);
                    it != end; it++)
                {
                    it->second.get()->outgoing_links_.clear();
                    std::vector<Vertex> neighbor_list;
                    getNeighborList(neighbor_list, it->second.get()->get_center_point(), 1);
                    
                    for (auto &it_tmp : neighbor_list) {
                        ADJ::iterator it_neighbor = adjacency_map_.find(it_tmp.get_name());
                        if (it_neighbor != adjacency_map_.end() /*&& !existsEdge(*it->second.get(), it_neighbor->second.get()->get_name())*/ ) {
                            double diff = it_neighbor->second.get()->get_height() - it->second.get()->get_height();
                            // diff <= threshold_height_
                            if (threshold_height_ - diff >= 1e-10) {
                                // std::cout << "Adding edge bw " << it->second.get()->get_name() << " - " << it_neighbor->second.get()->get_name() << std::endl;
                                addEdge(it, it_tmp.get_name());
                            }
                        }
                    }                   
                }
            }
            break;

        case 1:
            for (int i = adjacency_map_.bucket_count()/4, bucket_end = adjacency_map_.bucket_count()/2;
                    i < bucket_end; i++)
            {
                for (ADJ::local_iterator it = adjacency_map_.begin(i), end = adjacency_map_.end(i);
                    it != end; it++)
                {
                    it->second.get()->outgoing_links_.clear();
                    std::vector<Vertex> neighbor_list;
                    getNeighborList(neighbor_list, it->second.get()->get_center_point(), 1);
                    
                    for (auto &it_tmp : neighbor_list) {
                        ADJ::iterator it_neighbor = adjacency_map_.find(it_tmp.get_name());
                        if (it_neighbor != adjacency_map_.end() /*&& !existsEdge(*it->second.get(), it_neighbor->second.get()->get_name())*/ ) {
                            double diff = fabs(it_neighbor->second.get()->get_height() - it->second.get()->get_height());
                            if (threshold_height_ - diff >= 1e-10) {
                                // std::cout << "Adding edge bw " << it->second.get()->get_name() << " - " << it_neighbor->second.get()->get_name() << std::endl;
                                addEdge(it, it_tmp.get_name());
                            }
                        }
                    }                   
                }
            }
            break;

        case 2:
            for (int i = adjacency_map_.bucket_count()/2, bucket_end = 3*adjacency_map_.bucket_count() / 4;
                    i < bucket_end; i++)
            {
                for (ADJ::local_iterator it = adjacency_map_.begin(i), end = adjacency_map_.end(i);
                    it != end; it++)
                {
                    it->second.get()->outgoing_links_.clear();
                    std::vector<Vertex> neighbor_list;
                    getNeighborList(neighbor_list, it->second.get()->get_center_point(), 1);
                    
                    for (auto &it_tmp : neighbor_list) {
                        ADJ::iterator it_neighbor = adjacency_map_.find(it_tmp.get_name());
                        if (it_neighbor != adjacency_map_.end() /*&& !existsEdge(*it->second.get(), it_neighbor->second.get()->get_name())*/ ) {
                            double diff = it_neighbor->second.get()->get_height() - it->second.get()->get_height();
                            if (threshold_height_ - diff >= 1e-10) {
                                // std::cout << "Adding edge bw " << it->second.get()->get_name() << " - " << it_neighbor->second.get()->get_name() << std::endl;
                                addEdge(it, it_tmp.get_name());
                            }
                        }
                    }                   
                }
            }
            break;
        case 3:
        for (int i = 3*adjacency_map_.bucket_count()/4, bucket_end = adjacency_map_.bucket_count();
                i < bucket_end; i++)
        {
            for (ADJ::local_iterator it = adjacency_map_.begin(i), end = adjacency_map_.end(i);
                it != end; it++)
            {
                it->second.get()->outgoing_links_.clear();
                std::vector<Vertex> neighbor_list;
                getNeighborList(neighbor_list, it->second.get()->get_center_point(), 1);
                
                for (auto &it_tmp : neighbor_list) {
                    ADJ::iterator it_neighbor = adjacency_map_.find(it_tmp.get_name());
                    if (it_neighbor != adjacency_map_.end() /*&& !existsEdge(*it->second.get(), it_neighbor->second.get()->get_name())*/ ) {
                        double diff = it_neighbor->second.get()->get_height() - it->second.get()->get_height();
                        if (threshold_height_ - diff >= 1e-10) {
                                // std::cout << "Adding edge bw " << it->second.get()->get_name() << " - " << it_neighbor->second.get()->get_name() << std::endl;
                                addEdge(it, it_tmp.get_name());
                        }
                    }
                }                   
            }
        }
        break;

        default:
            std::cout << "Wrong _part input for parallel constructEdges\n";
    }
}

bool Graph::existsEdge(const Vertex& _vx_from, const std::string& _name) const {
    for (std::list<VertexPtr>::const_iterator it = _vx_from.outgoing_links_.begin(), end = _vx_from.outgoing_links_.end();
        it != end; it++)
    {
        if ( (*it).get()->get_name() == _name) 
            return true;
        

    }
    return false;
}

// optimize addition overhead do it once for loop condition
void Graph::getNeighborList(std::vector<Vertex>& _list, Point _point, int depth) {
    double end_x = _point.x + depth * vertex_res_;
    double end_y = _point.y + depth * vertex_res_;
    for (double i_x = _point.x - depth * vertex_res_; i_x <= end_x; i_x += vertex_res_)
        for (double i_y = _point.y - depth * vertex_res_; i_y <= end_y; i_y += vertex_res_) {
            if (i_x != _point.x || i_y != _point.y)
                _list.push_back(Vertex(i_x, i_y));
        }
}

// not used currently
Point Graph::getCentralizedPoint(Point _point) {
    double x, y;
    x = _point.x; y = _point.y;
    if (floor(x) == ceil(x)) x+= 1e-5;
    if (floor(y) == ceil(y)) y+= 1e-5;
    double x_centr = (floor(x)+ceil(x)) / 2;
    double y_centr = (floor(y)+ceil(y)) / 2;
    return Point(x_centr, y_centr);
}

double Graph::get_distance(const Point &_from, const Point &_to) const {
    double x = _to.x - _from.x;
    double y = _to.y - _from.y;
    return sqrt(x*x + y*y);
}

/*
Not used currently
Update when findPath(Vertex, Vertex) is done
const Vertex* Graph::findPath(const Point& _start, const Point& _end) {
    using std::cout; using std::endl;

    fpqueue<Vertex, std::vector<Vertex>, Vertex::vertex_comparator> open_fpqueue;
    std::unordered_set<std::string> close_set;

    Vertex tmp_vx(_start); // = new Vertex(getNearestPoint(_start));
    setToNearestPoint(tmp_vx);
    ADJ::iterator it_vx = adjacency_map_.find(tmp_vx.get_name());
    if (it_vx != adjacency_map_.end())
        cout << "FindPath1: Found " << it_vx->first << " H: " << it_vx->second->get_height() << endl;
    else {
        cout << "FindPath1: Not found: " << tmp_vx.get_name() << ". Returning..\n" << endl;
        return NULL;
    }
    Vertex* start_vertex = it_vx->second;
    start_vertex->g_cost_ = 0;
    start_vertex->h_cost_ = get_distance(start_vertex->get_center_point(), _end);;
    start_vertex->calculate_fcost();
    open_fpqueue.push(*start_vertex);
    const Vertex* tmp;

    while (open_fpqueue.size() != 0) {
        it_vx = adjacency_map_.find(open_fpqueue.top().get_name());
        open_fpqueue.pop();
        Vertex* current_vertex = it_vx->second;
        close_set.insert(current_vertex->get_name());
        tmp = current_vertex;

        cout << "\n\n@@@@@@@@@At name: " << current_vertex->get_name() << ", f: " << current_vertex->get_fcost() << endl;
        if (*current_vertex == _end) {       // path has been found
            cout << "[findPath]: Path's been found. Returning..\n";
            return current_vertex;
        }
        std::list<Vertex>& neighbor_list = current_vertex->outgoing_links_;             // may be change outgoing_links[Vertex] to [VERTEX*]
        cout  << "sizeofoutgoings: " << neighbor_list.size() << endl;
        for (std::list<Vertex>::iterator it = neighbor_list.begin(), end = neighbor_list.end(); it != end; it++) {
            if (close_set.find(it->get_name()) == close_set.end()) {          // did not traverse this vertex
                // if not in the open queue to be looked for
                Vertex* new_vx = adjacency_map_.find(it->get_name())->second;
                double new_gcost = current_vertex->g_cost_ + get_distance(current_vertex->get_center_point(), new_vx->get_center_point());
                if (!open_fpqueue.find(*it)) {
                    new_vx->g_cost_ = current_vertex->g_cost_ + get_distance(current_vertex->get_center_point(), new_vx->get_center_point());
                    new_vx->h_cost_ = get_distance(new_vx->get_center_point(), _end);
                    new_vx->calculate_fcost();
                    cout << "[DEBUG]: Putting to OPEN: " << *new_vx << " g_cost: " << new_vx->g_cost_ <<
                            ", h_cost: " << new_vx->h_cost_  << endl;
                    new_vx->predecessor_vertex_ = current_vertex;
                    open_fpqueue.push(*new_vx);
                }else if (new_gcost < new_vx->g_cost_) {             // if new g_cost is better renew the path
                    double new_gcost = current_vertex->g_cost_ + get_distance(current_vertex->get_center_point(), it->get_center_point());

                    cout << "[DEBUG]: Putting to OPEN cuz gcost is better: " << new_vx << "g: " << new_vx->g_cost_ << "-->"
                         << new_gcost << ", h: " << new_vx->h_cost_ << endl;
                    new_vx->g_cost_ = new_gcost;
                    new_vx->calculate_fcost();
                    new_vx->predecessor_vertex_ = current_vertex;
                    open_fpqueue.push(*new_vx);
                }
                else
                    cout << "Did not put (" << new_vx << ") to the OPEN because its path is longer\n";
            } else
                cout << "[fpath]: did not put (" << it->get_name() << ") to the OPEN because it's been traversed already\n";
        }

    }
    cout << "Could not find path;" << endl;
    return tmp;

}
*/

std::shared_ptr<Vertex> Graph::findPath(const Vertex& _start, const Vertex& _end) {
    std::cout << "~~~~LOCKING [findPath_1]\n";
    std::lock_guard<std::mutex> lock_graph(this->graph_mutex_);
    using std::cout; using std::endl;

    // std::unique_lock<std::mutex> lock_graph_1(graph_mutex_, std::defer_lock);
    std::cout << "~~~~LOCKED [findPath_1]\n";
    clearVertexPredecessors();

    fpqueue<Vertex, std::vector<Vertex>, Vertex::vertex_comparator> open_fpqueue;
    std::unordered_set<std::string> close_set;

    Vertex tmp_vx = getNearestVertexInGraph(_start);
    Vertex end_vx = getNearestVertexInGraph(_end);
    std::cout << "Nearest start vertex of " << _start << " = " << tmp_vx << std::endl;
    std::cout << "Nearest end vertex of " << _end << " = " << end_vx << std::endl;
    // std::cout << "~~~~LOCKING [findPath_1]\n";
    // lock_graph_1.lock();
    // std::cout << "~~~~LOCKED [findPath_1]\n";
    ADJ::iterator it_vx = adjacency_map_.find(tmp_vx.get_name());
    if (it_vx != adjacency_map_.end())
        cout << "FindPath1: Found " << it_vx->first << " H: " << it_vx->second.get()->get_height() << endl;
    else {
        cout << "FindPath1: Not found: " << tmp_vx.get_name() << ". Returning..\n\n\n" << endl;
        // lock_graph_1.unlock();
        return NULL;
    }
    VertexPtr start_vertex = it_vx->second;
    start_vertex.get()->g_cost_ = 0;
    start_vertex.get()->h_cost_ = get_distance(start_vertex.get()->get_center_point(), end_vx.get_center_point());;
    start_vertex.get()->calculate_fcost();
    open_fpqueue.push(*start_vertex.get());
    // lock_graph_1.unlock();
    //VertexPtr tmp = nullptr;

    while (open_fpqueue.size() != 0) {
        // std::unique_lock<std::mutex> lock_graph_2(graph_mutex_, std::defer_lock);
        // std::cout << "~~~~LOCKING [findPath_2]\n";
        // lock_graph_2.lock();
        // std::cout << "~~~~LOCKED [findPath_2]\n";
        it_vx = adjacency_map_.find(open_fpqueue.top().get_name());
        open_fpqueue.pop();
        VertexPtr current_vertex;
        if (it_vx != adjacency_map_.end()) {
            current_vertex = it_vx->second;
            close_set.insert(current_vertex->get_name());
        } else {
            // lock_graph_2.unlock();
            continue;
        }
        //tmp = current_vertex;
        // lock_graph_2.unlock();
       // cout << "\n\n@@@@@@@@@At name: " << current_vertex.get()->get_name() << ", f: " << current_vertex.get()->get_fcost() << endl;
        if (*current_vertex == end_vx) {       // path has been found
           // cout << "[findPath]: Path's been found. Returning..\n";
            return current_vertex;
        }
        std::list<VertexPtr>& neighbor_list = current_vertex.get()->outgoing_links_;            
        //cout  << "sizeofoutgoings: " << neighbor_list.size() << endl;
        for (std::list<VertexPtr>::iterator it = neighbor_list.begin(), end = neighbor_list.end(); it != end; it++) {
            if (close_set.find((*it).get()->get_name()) == close_set.end()) {          // did not traverse this vertex
                // if not in the open queue to be looked for
                it_vx = this->adjacency_map_.find((*it).get()->get_name());
                VertexPtr new_vx;
                if (it_vx != this->adjacency_map_.end())
                {
                    new_vx = it_vx->second;
                    double height_diff_ = 5 * fabs(current_vertex.get()->get_height() - new_vx.get()->get_height());
                    // std::cout << "Height diff: " << height_diff_ << std::endl;
                    double new_gcost = height_diff_ + current_vertex.get()->g_cost_ + get_distance(current_vertex.get()->get_center_point(), new_vx.get()->get_center_point());
                    if (!open_fpqueue.find( *(*it).get() ) ) {
                        new_vx.get()->g_cost_ = new_gcost;
                        new_vx.get()->h_cost_ = get_distance(new_vx.get()->get_center_point(), end_vx.get_center_point());
                        new_vx.get()->calculate_fcost();
                        //cout << "[DEBUG]: Putting to OPEN: " << *new_vx.get() << " g_cost: " << new_vx.get()->g_cost_ <<
                        //        ", h_cost: " << new_vx.get()->h_cost_ << ", f: " << new_vx.get()->get_fcost() << endl;
                        new_vx.get()->predecessor_vertex_ = current_vertex;
                        open_fpqueue.push(*new_vx.get());
                    }else if (new_gcost < new_vx.get()->g_cost_) {             // if new g_cost is better renew the path
                       // cout << "[DEBUG]: Putting to OPEN cuz gcost is better: " << new_vx << "g: " << new_vx->g_cost_ << "-->"
                       //      << new_gcost << ", h: " << new_vx->h_cost_ << ", f: " << new_vx->f_cost_ << endl;
                        new_vx->g_cost_ = new_gcost;
                        new_vx->calculate_fcost();
                        new_vx->predecessor_vertex_ = current_vertex;
                        open_fpqueue.push(*new_vx.get());
                    }
                    //else
                        //cout << "Did not put (" << new_vx.get() << ") to the OPEN because its path is longer\n";
                } //else
                        //cout << "Skipping " << (*it).get()->get_name() << std::endl;
           
            } //else
                //cout << "[fpath]: did not put (" << (*it).get()->get_name() << ") to the OPEN because it's been traversed already\n";
        }

    }
    cout << "~~~~~Could not find path;" << endl;
    return nullptr;

}

// IEEE standards state that round(-0.2) will return -0.0 which confronts
// our map key logic  1-0 == 1--0
void Graph::setToNearestPoint(Vertex& _v) {
    double x = std::round(_v.get_center_point().x*coeff_res_)/coeff_res_;
    double y = std::round(_v.get_center_point().y*coeff_res_)/coeff_res_;
    x == -0.0 ? x = 0. : x = x;
    y == -0.0 ? y = 0. : y = y;
    //Point * nearest = new Point(x, y);
    _v.set_center_point(Point(x,y));
   // return *nearest;
}

void Graph::setToNearestDiamond(Vertex& _v) {
    double x = std::round(_v.get_center_point().x / diamond_size_) * diamond_size_;
    double y = std::round(_v.get_center_point().y / diamond_size_) * diamond_size_;
    x == -0.0 ? x = 0. : x = x;
    y == -0.0 ? y = 0. : y = y;
    _v.set_center_point(Point(x, y));
    // std::cout << "Nearest Diamond: " << _v << std::endl;
}

void Graph::clearVertexPredecessors() {
    for (ADJ::iterator it = adjacency_map_.begin(), end = adjacency_map_.end();
        it != end; it++)
    {
        it->second.get()->predecessor_vertex_ = nullptr;
    }
}

double Graph::getGroundHeight() const {
    return this->ground_height_;
}

Vertex Graph::getNearestDiamond(const Vertex& _pos, bool& _empty) {
    if (this->diamond_vect_.size() == 0) {
        _empty = true;
        return Vertex(0., 0.);
    }
    int min_index = 0;
    double min_distance;
    std::unordered_map<std::string, Vertex>::iterator result_it = this->diamond_vect_.begin();
    min_distance = get_distance(result_it->second.get_center_point(), _pos.get_center_point());
    std::unordered_map<std::string, Vertex>::iterator it = this->diamond_vect_.begin();
    it++;
    for (std::unordered_map<std::string, Vertex>::iterator end = this->diamond_vect_.end(); it != end; it++) 
    {
        double distance = get_distance(it->second.get_center_point(), _pos.get_center_point());
        if (distance < min_distance) {
            min_distance = distance;
            result_it = it;
        }
    }
    std::cout << "Nearest diamond: " << result_it->second << std::endl;
    Vertex result = result_it->second;
    this->diamond_vect_.erase(result_it);
    this->visited_diamonds_.insert(result.get_name());
    return result;
}

bool Graph::isNearHole(const Vertex& _target_vx) const {
    for (ADJ::const_iterator it = this->hole_map_.begin(), end = this->hole_map_.end();
            it != end; ++it)
    {
        double distance = get_distance(it->second.get()->get_center_point(), _target_vx.get_center_point());
        if (distance < 0.1)
            return true;
    }
    return false;
}

