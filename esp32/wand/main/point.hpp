#ifndef _POINT_H_
#define _POINT_H_

#include <iostream>
#include <vector>
#include <map>

enum Axis {X_AXIS, Y_AXIS};

class Point {
    public:
        int x;
        int y;

        Point() {};
        Point(int x, int y): x(x), y(y) {};
        Point(Point const& other): x(other.x), y(other.y) {};
        int getAxis(Axis axis);
        bool operator==(Point const& other);
        bool operator!=(Point const& other);
        Point operator-(Point const& other);
        Point operator*(int mul) const;
        Point operator/(int div) const;
        Point& operator=(const Point&) = default;
};

std::ostream& operator<< (std::ostream &out, Point const& pt);

namespace std
{
    template<> struct less<Point>
    {
        bool operator() (const Point& lhs, const Point& rhs) const
        {
            if (lhs.y < rhs.y)
                return true;
            if ((lhs.y == rhs.y) & (lhs.x < rhs.x))
                return true;
            return false;
        }
    };
}

class PointBin {
    public:
        std::map<Point, time_t> bin;
        std::map<Point, int> bin_count;
        int bin_size;
        int half_bin_size;
        int bin_threshold;

        PointBin(int bin_size, int bin_threshold):
            bin_size(bin_size),
            half_bin_size(bin_size/2),
            bin_threshold(bin_threshold) {};
        inline Point bin_point(Point pt) {
            // return Point(pt.x/bin_size, pt.y/bin_size);
            int halfbin_x = (pt.x / half_bin_size) * half_bin_size;
            int halfbin_y = (pt.y / half_bin_size) * half_bin_size;
            int bin_x = (pt.x/bin_size) * bin_size;
            int bin_y = (pt.y/bin_size) * bin_size;

            int final_x, final_y;

            if ((halfbin_x - bin_x) < half_bin_size) {
                final_x = bin_x;
            } else {
                final_x = bin_x + bin_size;
            }

            if ((halfbin_y - bin_y) < half_bin_size) {
                final_y = bin_y;
            } else {
                final_y = bin_y + bin_size;
            }

            return Point(final_x/bin_size, final_y/bin_size);
        }
        bool contains(Point);
        void add(Point);
        bool remove(Point);
        int prune(int);
};

#endif // _POINT_H_