#include <iostream>
#include <vector>
#include <algorithm>

#include "blob_rect.hpp"
#include "point.hpp"

using namespace std;


bool pointSortX (Point a, Point b) {
    if (a.x == b.x)
        return a.y < b.y;
    return a.x < b.x;
}

bool pointSortY (Point a, Point b) {
    if (a.y == b.y)
        return a.x < b.x;
    return a.y < b.y;
}
typedef bool (*pointCompare)(Point, Point);

bool Rect::intersect(Rect const& other, Rect& merged) {
    merged.tl.x = max(tl.x, other.tl.x);
    merged.tl.y = max(tl.y, other.tl.y);
    merged.br.x = min(br.x, other.br.x);
    merged.br.y = min(br.y, other.br.y);

    if (merged.tl.x > merged.br.x || merged.tl.y > merged.br.y) {
        return false;
    }

    return true;
}

bool Rect::contains(int x, int y) {
    if ((x < tl.x) || (x > br.x)) {
        return false;
    }
    if ((y < tl.y) || (y > br.y)) {
        return false;
    }
    return true;
}

bool Rect::contains(Point pt) {
    return contains(pt.x, pt.y);
}

bool Rect::empty() {
    if ((br.x-tl.x == 0) || (br.y-tl.y == 0)) {
        return true;
    }
    return false;
}

Point Rect::center()
{
    return Point((tl.x+br.x)/2, (tl.y+br.y)/2);
}

ostream& operator<< (ostream &out, Rect const& rect) {
    out << "Rect(" << rect.tl << "," << rect.br << ")";
    return out;
}

// framePoints will be modified here!  The points will be sorted several times!
size_t rectsProjectedFromAxis(vector<Point>& framePoints, vector<Rect>& blobRects, Axis majorAxis) {

    Axis minorAxis = (majorAxis == X_AXIS ? Y_AXIS: X_AXIS);
    pointCompare majorAxisSort;
    pointCompare minorAxisSort;

    if (majorAxis == X_AXIS){
        majorAxisSort = pointSortX;
        minorAxisSort = pointSortY;
    } else {
        majorAxisSort = pointSortY;
        minorAxisSort = pointSortX;
    }

    std::sort(framePoints.begin(), framePoints.end(), majorAxisSort);

    blobRects.clear();

    int majorIdx, prevMajorIdx, prevMajorBreak;
    int minorIdx, prevMinorIdx, prevMinorBreak;

    prevMajorBreak = -1;
    vector<int> majorBreaks;
    for (majorIdx=0; majorIdx<framePoints.size(); majorIdx++) {
        if ((framePoints[majorIdx].getAxis(majorAxis) - prevMajorBreak) > 10) {
            majorBreaks.push_back(majorIdx);
        }
        prevMajorBreak = framePoints[majorIdx].getAxis(majorAxis);
    }
    majorBreaks.push_back(framePoints.size());

    prevMajorIdx = 0;
    for (majorIdx=0; majorIdx<majorBreaks.size(); majorIdx++) {
        if (majorBreaks[majorIdx] == prevMajorIdx)
            continue;

        int minMajor = framePoints[prevMajorIdx].getAxis(majorAxis);
        int maxMajor = framePoints[majorBreaks[majorIdx]-1].getAxis(majorAxis);

        sort(
            framePoints.begin() + prevMajorIdx,
            framePoints.begin() + majorBreaks[majorIdx],
            minorAxisSort
        );

        prevMinorBreak = -1;
        prevMinorIdx = prevMajorIdx;
        minorIdx=prevMajorIdx;
        for (; minorIdx<majorBreaks[majorIdx]; minorIdx++) {
            if ((framePoints[minorIdx].getAxis(minorAxis) - prevMinorBreak) > 10) {
                if (minorIdx != prevMinorIdx) {
                    if (majorAxis == X_AXIS) {
                        blobRects.push_back(
                            Rect(
                                Point(minMajor, framePoints[prevMinorIdx].getAxis(minorAxis)),
                                Point(maxMajor, framePoints[minorIdx-1].getAxis(minorAxis))
                            )
                        );
                    } else {
                        blobRects.push_back(
                            Rect(
                                Point(framePoints[prevMinorIdx].getAxis(minorAxis), minMajor),
                                Point(framePoints[minorIdx-1].getAxis(minorAxis), maxMajor)
                            )
                        );
                    }
                    prevMinorIdx = minorIdx;
                }
            }
            prevMinorBreak = framePoints[minorIdx].getAxis(minorAxis);
        }
        if (majorAxis == X_AXIS) {
            blobRects.push_back(
                Rect(
                    Point(minMajor, framePoints[prevMinorIdx].getAxis(minorAxis)),
                    Point(maxMajor, framePoints[minorIdx-1].getAxis(minorAxis))
                )
            );
        } else {
            blobRects.push_back(
                Rect(
                    Point(framePoints[prevMinorIdx].getAxis(minorAxis), minMajor),
                    Point(framePoints[minorIdx-1].getAxis(minorAxis), maxMajor)
                )
            );
        }
        prevMajorIdx = majorBreaks[majorIdx];
    }

    return blobRects.size();
}

// framePoints will be modified here!  The points will be sorted several times!
size_t findBlobRects(vector<Point>& framePoints, vector<Rect>& blobRects) {
    vector<Rect> rectsX, rectsY;
    vector<Rect>::iterator itX, itY;
    Rect interesction;

    blobRects.clear();

    rectsProjectedFromAxis(framePoints, rectsX, X_AXIS);
    rectsProjectedFromAxis(framePoints, rectsY, Y_AXIS);

    for (itX = rectsX.begin(); itX < rectsX.end(); itX++) {
        for (itY = rectsY.begin(); itY < rectsY.end(); itY++) {
            if ((*itX).intersect(*itY, interesction)) {
                blobRects.push_back(interesction);
            }
        }
    }
    return blobRects.size();
}
