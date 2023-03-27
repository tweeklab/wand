#include <iostream>
#include <vector>
#include <map>
#include <algorithm>

#include "point.hpp"

using namespace std;

// Point
int Point::getAxis(Axis axis) {
    if (axis == X_AXIS)
        return x;
    return y;
}

bool Point::operator==(Point const& other) {
    return (x == other.x) && (y == other.y);
}

bool Point::operator!=(Point const &other)
{
    return (x != other.x) || (y != other.y);
}

Point Point::operator-(Point const &other)
{
    return Point((x-other.x), (y-other.y));
}

Point Point::operator*(int mul) const
{
    return Point((x*mul), (y*mul));
}

Point Point::operator/(int div) const
{
    return Point((x/div), (y/div));
}

ostream& operator<< (ostream &out, Point const& pt) {
    out << "Point(" << pt.x << "," << pt.y << ")";
    return out;
}

// PointBin
bool PointBin::contains(Point pt) {
    if (bin.count(bin_point(pt)) > 0) {
        bin[bin_point(pt)] = time(NULL);
        if (bin_count[bin_point(pt)] > bin_threshold) {
            return true;
        }
    }
    return false;
}

void PointBin::add(Point pt) {
    bin[bin_point(pt)] = time(NULL);
    bin_count[bin_point(pt)]++;
}

bool PointBin::remove(Point pt) {
    bin_count.erase(bin_point(pt));
    return (bin.erase(bin_point(pt)) > 0);
}

int PointBin::prune(int expire_seconds) {
    vector<Point> old_points;
    int cutoff_time = time(NULL) - expire_seconds;
    for (auto it = bin.begin(); it != bin.end(); it++) {
        if (it->second < cutoff_time) {
            old_points.push_back(it->first);
        }
    }
    for (auto it = old_points.begin(); it != old_points.end(); it++) {
        bin.erase(*it);
        bin_count.erase(*it);
    }
    return old_points.size();
}
