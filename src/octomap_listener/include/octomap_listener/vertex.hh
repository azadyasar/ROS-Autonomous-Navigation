#ifndef VERTEX_HPP
#define VERTEX_HPP

#include "octomap_listener/point.hh"
#include <list>
#include <string>
#include <memory>

class Vertex {
    typedef std::shared_ptr<Vertex> VertexPtr;
private:

    std::string name_;
    double height_;
    Point center_point_;

public:
    std::list<VertexPtr> outgoing_links_;
    VertexPtr predecessor_vertex_;
    double g_cost_;
    double h_cost_;
    double f_cost_;
    int cell_size_;

    Vertex(double, double);
    Vertex(const Point&);
    Vertex(const Vertex&);
    Point get_center_point() const;
    void set_center_point(const Point&);
    std::string get_name() const;
    double get_fcost() const;
    void calculate_fcost();
    double get_height() const;
    void set_height(double);
    bool operator<(const Vertex&) const;
    bool operator==(const Vertex& ) const;
    bool operator ==(const Point& ) const;
    Vertex& operator =(const Vertex&);
    Vertex& operator =(const Vertex*);
    // TODO
    struct vertex_comparator {
     bool operator()(const Vertex& _v1, const Vertex& _v2) {
        return (_v1.get_fcost() > _v2.get_fcost());
     }
    };

    struct vertex_comparator_map {
        bool operator()(const Vertex& _v1, const Vertex& _v2) {
            return (_v1.get_fcost() < _v2.get_fcost());
        }
    };

    struct vertexPtr_comparator
    {
      bool operator()(Vertex* const& lhs, Vertex* const& rhs)   {
        return (lhs->get_center_point().x == rhs->get_center_point().x &&
                lhs->get_center_point().y == rhs->get_center_point().y);
      }
    };

};

std::ostream& operator<<(std::ostream&, const Vertex&);
std::ostream& operator<<(std::ostream&, const Vertex*);

namespace std
{
    template <>
    struct hash<Vertex>
    {
        std::size_t operator()(const Vertex& k) const
        {
            return std::hash<std::string>()(k.get_name());
        }
    };

    template <>
    struct hash<Vertex*>
    {
        std::size_t operator()(const Vertex* k) const
        {
            return std::hash<std::string>()(k->get_name());
        }
    };
}

#endif // Vertex_HPP
