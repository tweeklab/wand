#ifndef _BLOB_RECT_H_
#define _BLOB_RECT_H_

#include <iostream>
#include <vector>

#include "point.hpp"

class Rect {
    public:
        Point tl; // Top-left
        Point br; // Bottom-right

        Rect() {};
        Rect(Rect const& other):
            tl(other.tl),
            br(other.br){};
        Rect(Point tl, Point br):
            tl(tl),
            br(br) {};

        bool intersect(Rect const& other, Rect& merged);
        bool contains(Point pt);
        bool contains(int x, int y);
        bool empty();
        Point center();
};

std::ostream& operator<< (std::ostream &out, Rect const& rect);

bool pointSortX (Point a, Point b);
bool pointSortY (Point a, Point b);
size_t findBlobRects(std::vector<Point>& framePoints, std::vector<Rect>& blobRects);

#endif // _BLOB_RECT_H_