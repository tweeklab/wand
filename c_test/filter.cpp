#include <iostream>
#include <vector>
#include <chrono>
#include <deque>
#include <algorithm>
#include <random>
#include <unistd.h>

#include "blob_rect.hpp"
#include "point.hpp"
#include "test_frames.hpp"
#include "atan_lut.hpp"
#include "cos_lut.hpp"

using namespace std;

#define RAW_FRAMES_SIZE 50
#define FILTER_QUEUE_SIZE 3
#define FRAME_QUEUE_HW_MARK 10
#define IDLE_FRAME_COUNT 30
#define LOSER_BIN_BINSIZE 20
#define LOSER_BIN_LIFETIME_SECONDS 30

using namespace std;

typedef enum {
    IDLE
} filter_state_t;

template <class baseType>
size_t bounded_deque_push_back(deque<baseType>& q, baseType x, size_t maxSize) {
    size_t purge_count = 0;
    q.push_back(x);
    while (q.size() > maxSize) {
        q.pop_front();
        purge_count++;
    }
    return purge_count;
}

int atan2_lut(int y, int x) {
    int lookup_val;
    uint8_t flip_yx = 0;
    uint8_t y_neg = 0;
    uint8_t x_neg = 0;
    if (x < 0) {
        x_neg = 1;
        x = -x;
    }
    if (y < 0) {
        y_neg = 1;
        y = -y;
    }
    if (y > x) {
        int tmp = y;
        y = x;
        x = tmp;
        flip_yx = 1;
    }

    if (x == 0) {
        lookup_val = 0;
    } else {
        lookup_val = atan_lut_64[(y * 64)/x];
    }
    
    if (flip_yx) {
        lookup_val = 90 - lookup_val;
    }
    if (x_neg) {
        lookup_val = 180 - lookup_val;
    }
    if (y_neg) {
        lookup_val = -lookup_val;
    }

    return lookup_val;
}

int det(Point v1, Point v2) {
    return (v1.x*v2.y) - (v1.y*v2.x);
}

int dot(Point v1, Point v2) {
    return (v1.x*v2.x) + (v1.y*v2.y);
}

deque<vector<Point>> detect_queue;
int idle_counter;
int frame_counter;
filter_state_t filter_state;
PointBin loser_bin(LOSER_BIN_BINSIZE, 10);

// Bound by FILTER_QUEUE_SIZE
deque<vector<Point>> filter_queue; // Recent frames
vector<Point> path_point_queue; // Winning points

void append_to_path_point_queue(Point pt) {
    if (path_point_queue.size() == 0 || path_point_queue.back() != pt) {
        path_point_queue.push_back(pt);
    }
}

void filter() {
    int min_sum = INT32_MAX;
    int winner_sum = 0;
    vector<Point> winner_points;
    Point winner(-1, -1);
    // Build test grids
    for (auto it1 = filter_queue[0].begin(); it1 != filter_queue[0].end(); it1++) {
        if (loser_bin.contains(*it1))
            continue;
        for (auto it2 = filter_queue[1].begin(); it2 != filter_queue[1].end(); it2++) {
            if (loser_bin.contains(*it2))
                continue;
            for (auto it3 = filter_queue[2].begin(); it3 != filter_queue[2].end(); it3++) {
                if (loser_bin.contains(*it3))
                    continue;
                Point v0 = *it1 - path_point_queue.back();
                Point v1 = *it2 - *it1;
                Point v2 = *it3 - *it2;
                if (*it1 == path_point_queue.back() || *it1 == *it2 || *it2 == *it3) {
                    loser_bin.add(*it1);
                    continue;
                }
                int angle1 = atan2_lut(det(v0, v1), dot(v0, v1));
                int angle2 = atan2_lut(det(v1, v2), dot(v1, v2));
                int sum = abs(angle1) + abs(angle2);
                min_sum = std::min(min_sum, sum);
                if (min_sum == sum) {
                    winner_points = {path_point_queue.back(), *it1, *it2, *it3};
                    winner = *it1;
                    winner_sum = sum;
                }
            }
        }
    }
    if (winner.x > 0 && winner.y > 0) {
        append_to_path_point_queue(winner);
    }
    for (auto it1 = filter_queue[0].begin(); it1 != filter_queue[0].end(); it1++) {
        if (*it1 != winner) {
            loser_bin.add(*it1);
        }
    }
}

void process_single_frame(vector<Point> frame) {
    // Yeah the frames are filtered when the first come in to
    // handle_incoming_frame, but each time we run the detect
    // queue it's better to apply current knowledge of losers
    // to all frames in the queue that came in earlier.
    vector<Point> tmp_pts;
    for (auto pit = frame.begin(); pit < frame.end(); pit++) {
        if (!loser_bin.contains(*pit)) {
            tmp_pts.push_back(*pit);
        }
    }
    if (tmp_pts.size() == 0) {
        return;
    }

    bounded_deque_push_back(filter_queue, tmp_pts, FILTER_QUEUE_SIZE);
    // Need all 3 frames in the queue to vote on a good point
    if (filter_queue.size() != 3) {
        return;
    }
    if (filter_queue[0].size() == 1) {
        // Only one point in the frame, no ambiguity so just pass it
        // along to the path point queue.
        append_to_path_point_queue(filter_queue[0][0]);
    } else {
        if (path_point_queue.size() > 0) {
            filter();
        } else {
            Point random_point;
            random_point = filter_queue[0][rand() % filter_queue[0].size()];
            append_to_path_point_queue(random_point);
        }
    }
}


void handle_incoming_frame(vector<Point> frame) {
    vector<Point> tmp_pts;
    for (auto pit = frame.begin(); pit < frame.end(); pit++) {
        if (!loser_bin.contains(*pit)) {
            tmp_pts.push_back(*pit);
        }
    }
    if (tmp_pts.size() == 0) {
        if (idle_counter < IDLE_FRAME_COUNT) {
            ++idle_counter;
        }
    } else {
        bounded_deque_push_back(detect_queue, tmp_pts, RAW_FRAMES_SIZE);
    }

    if (idle_counter >= IDLE_FRAME_COUNT) {
        filter_state = IDLE;
        detect_queue.clear();
    }

    if ((++frame_counter % FRAME_QUEUE_HW_MARK) == 0) {
        path_point_queue.clear();
        filter_queue.clear();
        for (auto fit = detect_queue.begin(); fit != detect_queue.end(); fit++) {
            process_single_frame(*fit);
        }
    }
}

int norm_lut(int y, int x) {
    int angle;

    if (x < 0) {
        x = -x;
    }
    if (y < 0) {
        y = -y;
    }
    if (y > x) {
        int tmp = y;
        y = x;
        x = tmp;
    }

    if (x == 0) {
        angle = 0;
    } else {
        angle = atan_lut_64[(y * 64)/x];
    }

    return (x*32768) / cos_lut[angle];
}

int main() {
    int total = 0;

    auto t1 = std::chrono::high_resolution_clock::now();
    for (auto i = 0; i < 101; i++) {
        total += norm_lut(i, 100);
    }

    for (auto i = 100; i >= 0; i--) {
        total += norm_lut(100, i);
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> ms_double = t2 - t1;
    cout << total << endl;
    cout << "Total time: " << ms_double.count() << "ms" << endl;
}

// int main() {
//     vector<std::chrono::duration<double, std::milli>> all_frame_times;

//     idle_counter = 0;
//     frame_counter = 0;
//     srand(42);

//     cout << test_frame_stream.size() << endl;

//     for (auto fit = test_frame_stream.begin(); fit != test_frame_stream.end(); fit++) {
//         auto t1 = std::chrono::high_resolution_clock::now();
//         handle_incoming_frame(*fit);
//         auto t2 = std::chrono::high_resolution_clock::now();
//         std::chrono::duration<double, std::milli> ms_double = t2 - t1;
//         all_frame_times.push_back(ms_double);
//     }

//     auto min_time = min_element(all_frame_times.begin(), all_frame_times.end());
//     cout << "Min frame time: " << (*min_time).count() << "ms" << endl;
//     auto max_time = max_element(all_frame_times.begin(), all_frame_times.end());
//     cout << "Max frame time: " << (*max_time).count() << "ms" << endl;

//     for (auto it = path_point_queue.begin(); it != path_point_queue.end(); it++) {
//         cout << *it << "," << endl;
//     }
//     // cout << "==============" << endl;
//     // for (auto it = loser_bin.bin_count.begin(); it != loser_bin.bin_count.end(); it++) {
//     //     cout << it->first * loser_bin.bin_size << " " << it->second << endl;
//     // }
//     cout << "==============" << endl;
//     for (auto it = loser_bin.bin.begin(); it != loser_bin.bin.end(); it++) {
//         if (loser_bin.bin_count[it->first] < loser_bin.bin_threshold)
//             continue;
//         cout << it->first * loser_bin.bin_size << "," << endl;
//     }

//     return 0;
// }
