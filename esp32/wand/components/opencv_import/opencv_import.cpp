#include <stdio.h>
#include "opencv_import.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/matx.hpp"
#include "opencv2/features2d.hpp"

// cv::Mat src_gray(320, 200, CV_8UC1);
// cv::Mat src_stuff(320, 200, CV_8UC1);
std::vector < std::vector<cv::Point> > contours;

void opencv_func(uint8_t *data)
{
    // src_gray = 5;
    cv::Mat src_mat = cv::Mat(176, 144, CV_8UC1, data);
    // printf("Stuff: %0X\n", src_mat.at<uint>(30, 30));
    // cv::threshold(src_gray, src_stuff, 6, 255, cv::THRESH_BINARY);
    // cv::findContours(src_mat, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    cv::SimpleBlobDetector::Params params;
    params.minThreshold = 100;
    params.maxThreshold = 101;
    params.thresholdStep = 1;
    // params.minDistBetweenBlobs = 3;
    params.minRepeatability = 1;
    params.filterByColor = true;
    params.blobColor = 255;
    params.filterByArea = true;
    params.minArea = 2;
    params.filterByCircularity = false;
    params.minCircularity = 0.3;
    params.filterByConvexity = false;
    params.minConvexity = 0.87;
    params.filterByInertia = false;
    params.minInertiaRatio = 0.01;    

    // Set up the detector with default parameters.
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    // Detect blobs.
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(src_mat, keypoints);
    if (keypoints.size() > 0) {
        printf("Keypoints: ");
        for (size_t i = 0; i < keypoints.size(); i++) {
            printf("(%f, %f, %f)", keypoints[i].pt.x, keypoints[i].pt.y, keypoints[i].size);
        }
        printf("\n");
    }

    // printf("Func called!!!\n");
    // for( size_t i = 0; i< contours.size(); i++ ) {
    //     printf("Coutour: %d, %d", contours[i][0].x, contours[i][0].y);
    // }
    // printf("Contours: %d\n", contours.size());
    contours.clear();
}
