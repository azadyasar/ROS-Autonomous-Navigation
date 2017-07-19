#include "octomap_listener/point.hh"
#include <iostream>

Point::Point() {}

Point::Point(double _x, double _y) {
    this->x = _x;
    this->y = _y;
}

std::ostream& operator <<(std::ostream& os, const Point& _p) {
    return os << _p.x << "-" << _p.y;
}
