#include <iostream>
#include <vector>
#include <chrono>

#include "blob_rect.hpp"

using namespace std;

int main() {
    vector<Point> rawPoints = {
        Point(141, 22),
        Point(138, 23),
        Point(137, 24),
        Point(138, 24),
        Point(138, 25),
        Point(138, 26),
        Point(139, 26),
        Point(140, 26),
        Point(141, 26),
        Point(139, 27),
        Point(140, 27),
        Point(165, 46),
        Point(166, 46),
        Point(166, 47),
        Point(165, 47),
        Point(116, 114),
        Point(117, 114),
        Point(116, 115),
        Point(117, 115),
        Point(118, 115),
        Point(116, 116),
        Point(117, 116),
        Point(118, 116),
        Point(116, 117),
        Point(117, 117),
        Point(128, 126),
        Point(126, 127),
        Point(127, 127),
        Point(128, 127),
        Point(129, 127),
        Point(130, 127),
        Point(131, 127),
        Point(132, 127),
        Point(133, 127),
        Point(134, 127),
        Point(135, 127),
        Point(136, 127),
        Point(125, 128),
        Point(126, 128),
        Point(127, 128),
        Point(128, 128),
        Point(129, 128),
        Point(130, 128),
        Point(131, 128),
        Point(132, 128),
        Point(133, 128),
        Point(134, 128),
        Point(135, 128),
        Point(136, 128),
        Point(137, 128),
        Point(138, 128),
        Point(139, 128),
        Point(125, 129),
        Point(126, 129),
        Point(127, 129),
        Point(128, 129),
        Point(129, 129),
        Point(130, 129),
        Point(131, 129),
        Point(132, 129),
        Point(133, 129),
        Point(134, 129),
        Point(135, 129),
        Point(136, 129),
        Point(137, 129),
        Point(138, 129),
        Point(139, 129),
        Point(124, 130),
        Point(125, 130),
        Point(126, 130),
        Point(127, 130),
        Point(128, 130),
        Point(129, 130),
        Point(130, 130),
        Point(131, 130),
        Point(132, 130),
        Point(133, 130),
        Point(134, 130),
        Point(135, 130),
        Point(136, 130),
        Point(137, 130),
        Point(138, 130),
        Point(139, 130),
        Point(140, 130),
        Point(125, 131),
        Point(126, 131),
        Point(127, 131),
        Point(128, 131),
        Point(133, 131),
        Point(134, 131),
        Point(135, 131),
        Point(136, 131),
        Point(137, 131),
        Point(138, 131),
        Point(139, 131),
        Point(140, 131),
        Point(141, 131),
        Point(134, 132),
        Point(135, 132),
        Point(136, 132),
        Point(137, 132),
        Point(138, 132),
        Point(139, 132),
        Point(140, 132),
        Point(141, 132),
        Point(142, 132),
        Point(136, 133),
        Point(137, 133),
        Point(138, 133),
        Point(139, 133),
        Point(140, 133),
        Point(141, 133),
        Point(142, 133),
        Point(143, 133),
        Point(122, 134),
        Point(136, 134),
        Point(137, 134),
        Point(138, 134),
        Point(139, 134),
        Point(140, 134),
        Point(141, 134),
        Point(142, 134),
        Point(143, 134),
        Point(122, 135),
        Point(126, 135),
        Point(127, 135),
        Point(128, 135),
        Point(129, 135),
        Point(130, 135),
        Point(131, 135),
        Point(132, 135),
        Point(133, 135),
        Point(134, 135),
        Point(136, 135),
        Point(137, 135),
        Point(138, 135),
        Point(139, 135),
        Point(140, 135),
        Point(141, 135),
        Point(142, 135),
        Point(126, 136),
        Point(127, 136),
        Point(128, 136),
        Point(129, 136),
        Point(130, 136),
        Point(131, 136),
        Point(132, 136),
        Point(133, 136),
        Point(134, 136),
        Point(135, 136),
        Point(136, 136),
        Point(137, 136),
        Point(138, 136),
        Point(139, 136),
        Point(140, 136),
        Point(141, 136),
        Point(142, 136),
        Point(120, 137),
        Point(121, 137),
        Point(122, 137),
        Point(123, 137),
        Point(124, 137),
        Point(126, 137),
        Point(127, 137),
        Point(128, 137),
        Point(129, 137),
        Point(130, 137),
        Point(131, 137),
        Point(132, 137),
        Point(133, 137),
        Point(134, 137),
        Point(135, 137),
        Point(136, 137),
        Point(137, 137),
        Point(138, 137),
        Point(139, 137),
        Point(140, 137),
        Point(141, 137),
        Point(142, 137),
        Point(123, 138),
        Point(124, 138),
        Point(127, 138),
        Point(128, 138),
        Point(129, 138),
        Point(130, 138),
        Point(131, 138),
        Point(132, 138),
        Point(133, 138),
        Point(134, 138),
        Point(135, 138),
        Point(136, 138),
        Point(137, 138),
        Point(138, 138),
        Point(139, 138),
        Point(134, 139),
        Point(135, 139),
        Point(136, 139),
        Point(137, 139),
        Point(138, 139),
        Point(139, 139),
        Point(139, 140),
        Point(140, 140),
        Point(141, 141),
        Point(142, 141)
    };

    vector<Rect> rects;

    using std::chrono::high_resolution_clock;
    using std::chrono::duration;

    auto t1 = high_resolution_clock::now();
    findBlobRects(rawPoints, rects);
    auto t2 = high_resolution_clock::now();
    duration<double, std::milli> ms_double = t2 - t1;

    std::cout << ms_double.count() << "ms\n";

    for (size_t i=0; i<rects.size(); i++) {
        cout << rects[i].center() << " " << rects[i].empty() << endl;
    }

    return 0;
}