#ifndef POINT_HPP
#define POINT_HPP

#include <iosfwd>

struct Point {
public:
    double x, y;
    Point();
    Point(double, double);
};

std::ostream& operator<<(std::ostream&, const Point&);

#endif // POINT_HPP
