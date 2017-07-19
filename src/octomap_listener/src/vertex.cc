#include <octomap_listener/vertex.hh>
#include <octomap_listener/point.hh>
#include <iostream>

// Predecessor_vertex pointer is assigned null otherwise it crashes with exit code -11
Vertex::Vertex(double _x, double _y) : predecessor_vertex_(nullptr), cell_size_(0)
{
    this->center_point_ = Point(_x, _y);
    this->name_ = std::to_string(_x) + "-" + std::to_string(_y);
    this->g_cost_ = this->h_cost_ = this->f_cost_ = 0;
}

Vertex::Vertex(const Point& _point) : predecessor_vertex_(nullptr), cell_size_(0)
{
    this->center_point_ = _point;
    this->name_ = std::to_string(_point.x) + "-" + std::to_string(_point.y);
    this->g_cost_ = this->h_cost_ = this->f_cost_ = 0;
}

Vertex::Vertex(const Vertex& _vertex) : predecessor_vertex_(nullptr), cell_size_(0)
{
    this->center_point_ = _vertex.center_point_;
    this->height_ = _vertex.get_height();
    this->name_ = std::to_string(this->center_point_.x) + "-" + std::to_string(this->center_point_.y);
    this->g_cost_ = _vertex.g_cost_;
    this->h_cost_ = _vertex.h_cost_;
    this->f_cost_ = _vertex.f_cost_;
    this->outgoing_links_ = _vertex.outgoing_links_;
    this->predecessor_vertex_ = _vertex.predecessor_vertex_;
}

Point Vertex::get_center_point()  const {
    return this->center_point_;
}

void Vertex::set_center_point(const Point& _p) {
    this->center_point_ = _p;
    this->name_ = std::to_string(_p.x) + "-" + std::to_string(_p.y);
}

std::string Vertex::get_name() const{
    return this->name_;
}

void Vertex::set_height(double _h) {
    this->height_ = _h;
}

void Vertex::calculate_fcost() {
    this->f_cost_ = this->g_cost_ + this->h_cost_;
}

double Vertex::get_height() const {
    return this->height_;
}

double Vertex::get_fcost() const {
    return this->f_cost_;
}

/** Used by set, map.
 * Vertex used in a set container which uses <  for equality
 * if ( a < b)
 * else if (b < a)
 * else put here
 * so we need a < that can distinguish two diferent Vertices
 * */
bool Vertex::operator <(const Vertex& rhs) const{
    return (this->center_point_.x < rhs.center_point_.x || this->center_point_.y < rhs.center_point_.y);
}

/** Used by unordered_map, unordered_set.
 * @brief Vertex::operator ==
 * @param rhs
 * @return bool
 */
bool Vertex::operator==(const Vertex& rhs) const {
    return (this->get_center_point().x == rhs.get_center_point().x && this->get_center_point().y == rhs.get_center_point().y);
}

bool Vertex::operator ==(const Point& rhs) const {
    return (this->get_center_point().x == rhs.x && this->get_center_point().y == rhs.y);
}

Vertex& Vertex::operator =(const Vertex& _rhs) {
    this->set_center_point(_rhs.get_center_point());
    this->g_cost_ = _rhs.g_cost_;
    this->h_cost_ = _rhs.h_cost_;
    this->f_cost_ = _rhs.f_cost_;
    this->set_height(_rhs.get_height());
    this->outgoing_links_ = _rhs.outgoing_links_;
    this->predecessor_vertex_ = _rhs.predecessor_vertex_;
    this->cell_size_ = _rhs.cell_size_;
    return *this;
}

Vertex& Vertex::operator =(const Vertex* _rhs) {
    this->set_center_point(_rhs->get_center_point());
    this->g_cost_ = _rhs->g_cost_;
    this->h_cost_ = _rhs->h_cost_;
    this->f_cost_ = _rhs->f_cost_;
    this->set_height(_rhs->get_height());
    this->outgoing_links_ = _rhs->outgoing_links_;
    this->predecessor_vertex_ = _rhs->predecessor_vertex_;
    this->cell_size_ = _rhs->cell_size_;
    return *this;
}

std::ostream& operator <<(std::ostream& os, const Vertex& v) {
    os << "(" << v.get_center_point().x << ")-(" << v.get_center_point().y << "): Height: " << v.get_height()
        << ", fcost: " << v.get_fcost();
    return os;
}

std::ostream& operator <<(std::ostream& os, const Vertex* v) {
    os << "(" << v->get_center_point().x << ")-(" << v->get_center_point().y << "): Height: " << v->get_height()
        <<", fcost: " << v->get_fcost();
    return os;
}



