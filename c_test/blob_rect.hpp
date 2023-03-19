#ifndef _BLOB_RECT_H_
#define _BLOB_RECT_H_

#include <iostream>
#include <vector>

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
};

std::ostream& operator<< (std::ostream &out, Point const& pt);

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
        bool empty();
        Point center();
};

std::ostream& operator<< (std::ostream &out, Rect const& rect);

size_t findBlobRects(std::vector<Point>& framePoints, std::vector<Rect>& blobRects);

#endif // _BLOB_RECT_H_