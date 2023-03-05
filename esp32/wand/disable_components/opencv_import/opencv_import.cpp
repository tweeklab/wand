// #include <stdio.h>
#include <vector>
#include "opencv_import.hpp"
#include "blob_rect.h"
// #include "opencv2/imgproc.hpp"
// #include "opencv2/core/types.hpp"
// #include "opencv2/core/mat.hpp"
// #include "opencv2/core/matx.hpp"
// #include "opencv2/features2d.hpp"

#include "opencv2/core.hpp"

void opencv_func(std::vector<point>& zero_points, std::vector<Point>& out_centers)
{
    cv::Point2f tmp;
    point outtmp;
    std::vector<cv::Point2f> centers, inputs;
    int spanx, spany, clusters, minx=320, miny=240, maxx=0, maxy=0;

    out_centers.clear();
    if (zero_points.size() == 0)
        return;

    for (size_t i = 0; i < zero_points.size(); i++) {
        tmp.x = zero_points[i].x;
        tmp.y = zero_points[i].y;
        minx = MIN(minx, tmp.x);
        maxx = MAX(maxx, tmp.x);
        miny = MIN(miny, tmp.y);
        maxy = MAX(maxy, tmp.y);
        inputs.push_back(tmp);
    }

    spanx = maxx-minx;
    spany = maxy-miny;
    clusters = (spanx/20) + (spany/20);

    cv::Mat labels;

    int clusterCount = MAX(MIN(inputs.size(), clusters), 1);
    cv::kmeans(inputs, clusterCount, labels,
            cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, 1.0),
            3, cv::KMEANS_PP_CENTERS, centers);

    for (size_t i = 0; i < centers.size(); i++) {
        outtmp.x = centers[i].x;
        outtmp.y = centers[i].y;
        out_centers.push_back(outtmp);
    }
}