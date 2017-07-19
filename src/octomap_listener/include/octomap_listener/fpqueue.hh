#ifndef FPQUEUE_H
#define FPQUEUE_H

#include <queue>

/**
 * Extends priority queue and adds find method for open priority queue of A* algorithm
 */

template<
    typename T,
    typename Container = std::vector<T>,
    typename Compare = std::less<typename Container::value_type>
>
class fpqueue : public std::priority_queue<T, Container, Compare> {
public:
    typedef typename std::priority_queue<
    T,
    Container,
    Compare>::container_type::const_iterator const_iterator;

    bool find(const T& val) const {
        auto first = this->c.cbegin();
        auto last = this->c.cend();
        while (first != last) {
            if (*first == val) return true;
            ++first;
        }
        return false;
    }

    bool find(const T* val) const {
        auto first = this->c.cbegin();
        auto last = this->c.cend();
        while (first != last) {
            if (*first == *val) return true;
            ++first;
        }
        return false;
    }


};


#endif // FPQUEUE_H
