#include <stdio.h>
#include <cstring>
#include <vector>
#include <deque>
#include <algorithm>
#include <utility>
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wifi.h"
#include "JPEGDEC.h"
#include "blob_rect.hpp"
#include "point.hpp"
#include "atan_lut.hpp"
#include "cos_lut.hpp"
#include <sys/socket.h>
#include <netdb.h>            // struct addrinfo

using namespace std;

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define LED_GPIO_NUM      4

static const char *TAG = "wand";

uint8_t *out = NULL;
JPEGDEC jpeg;

#define OUT_BUFLEN 2048

#define RAW_FRAMES_SIZE 100
#define FILTER_QUEUE_SIZE 3
#define VELOCITY_QUEUE_SIZE 10
#define FRAME_QUEUE_HW_MARK 10
#define IDLE_FRAME_COUNT 30
#define LOSER_BIN_BINSIZE 20
#define LOSER_BIN_THRESHOLD 10
#define LOSER_BIN_LIFETIME_SECONDS 10

typedef enum {
    IDLE,
    ACTIVE,
    DWELL,
    COMMIT
} filter_state_t;

deque<vector<Point>> detect_queue;
int idle_counter;
int frame_counter;
filter_state_t filter_state;
PointBin loser_bin(LOSER_BIN_BINSIZE, LOSER_BIN_THRESHOLD);
PointBin reduce_bin(10, 0);

// Bound by FILTER_QUEUE_SIZE
deque<vector<Point>> filter_queue; // Recent frames
vector<Point> path_point_queue; // Winning points
deque<Point> velocity_queue; // Deque tracking recent points for velocity measurement
int v;

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

// Lookup table-based 2D norm calculation
int norm_lut(Point v) {
    int angle;
    int x = v.x;
    int y = v.y;

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

int det(Point v1, Point v2) {
    return (v1.x*v2.y) - (v1.y*v2.x);
}

int dot(Point v1, Point v2) {
    return (v1.x*v2.x) + (v1.y*v2.y);
}

void append_to_path_point_queue(Point pt) {
    if (path_point_queue.size() == 0 || path_point_queue.back() != pt) {
        path_point_queue.push_back(pt);
    }
}

int compute_velocity_queue_displacement(void) {
    auto offset_it = velocity_queue.begin() + 1;
    auto it = velocity_queue.begin();
    Point vsum(0, 0);
    if (velocity_queue.size() < 5) {
        return 0;
    }
    for (;offset_it != velocity_queue.end();it++, offset_it++) {
        // cout << *it << "->" << *offset_it << endl;
        vsum  = vsum + (*offset_it - *it);
    }
    return norm_lut(vsum);
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
                int angle1 = atan2_lut(det(v0, v1), dot(v0, v1)) * norm_lut(v0);
                int angle2 = atan2_lut(det(v1, v2), dot(v1, v2)) * norm_lut(v1);
                int sum = (abs(angle1) + abs(angle2));
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
        bounded_deque_push_back(velocity_queue, winner_points[3], VELOCITY_QUEUE_SIZE);
    }
    for (auto it1 = filter_queue[0].begin(); it1 != filter_queue[0].end(); it1++) {
        if (*it1 != winner) {
            // Points that bin with the winner are ignored.  This prevents
            // double-hits near the winning point from causing the winner
            // to convert to a loser.  It also helps prevent the winning point
            // from converting to a loser during the dwell time because
            // it will start showing up in ever successive frame
            loser_bin.add(*it1, winner);
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
        bounded_deque_push_back(velocity_queue, filter_queue[0][0], VELOCITY_QUEUE_SIZE);
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
    int dwell_start_frame = 0;

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
        idle_counter = 0;
    }

    if (idle_counter >= IDLE_FRAME_COUNT) {
        filter_state = IDLE;
        detect_queue.clear();
        velocity_queue.clear();
    }

    if ((++frame_counter % FRAME_QUEUE_HW_MARK) == 0) {
        path_point_queue.clear();
        filter_queue.clear();
        for (auto fit = detect_queue.begin(); fit != detect_queue.end(); fit++) {
            process_single_frame(*fit);
        }

        v = compute_velocity_queue_displacement();
        if (v > 20) {
            filter_state = ACTIVE;
        } else if (filter_state == ACTIVE && v < 20) {
            filter_state = DWELL;
            dwell_start_frame = frame_counter;
        } else if (filter_state == DWELL && path_point_queue.size() > 10) {
            if ((frame_counter - dwell_start_frame) > 60) {
                filter_state = COMMIT;
            }
        }
    }
}

#define HOST_IP "10.0.0.30"
#define PORT 3333
static int do_connect(void)
{
        const char *host_ip = HOST_IP;
        struct sockaddr_in dest_addr;
        inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);

        int sock =  socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            return -1;
        }
        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            return -1;
        }

        int val = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &val, sizeof(val));
        
        ESP_LOGI(TAG, "Successfully connected");

        return sock;
}

typedef struct {
    int height;
    int width;
    std::vector<Point> zero_points;
} user_data_t;

int drawMCUs(JPEGDRAW *pDraw)
{
    uint8_t *pIn = (uint8_t *)pDraw->pPixels;
    user_data_t *user = (user_data_t *)pDraw->pUser;
    uint8_t pixel;

    int top = pDraw->y * user->width;
    int bottom = top + (pDraw->iHeight * user->width);

    for (int y = top; y < bottom; y+=user->width) {
        for (int x = 0; x < pDraw->iWidth; x++) {
            pixel = (pIn[x] > 2 ? 255 : 0);
            if (pixel == 0) {
                if (user->zero_points.size() < 512) {
                    user->zero_points.push_back(Point(pDraw->x + x, y/user->width));
                }
            }
        }
        pIn += pDraw->iWidth;
    }
    return 1;
}

bool scale(std::vector<Point> const& orig, std::vector<Point>& scaled) {
    pair minMaxX = std::minmax_element(orig.begin(), orig.end(), pointSortX);
    pair minMaxY = std::minmax_element(orig.begin(), orig.end(), pointSortY);
    int xSize = minMaxX.second->x - minMaxX.first->x;
    int ySize = minMaxY.second->y - minMaxY.first->y;
    if (xSize < 40 || ySize < 30)
        return false;
    int scaleFactor = 10;
    if ((ySize*100)/296 < (xSize*100)/400) {
        // Scale X axis
        scaleFactor = 1 + (10*xSize)/400;
    } else {
        // Scale Y axis
        scaleFactor = 1 + (10*ySize)/296;
    }

    for (auto it = orig.begin(); it != orig.end(); it++) {
        Point orig_point = *it;
        orig_point.x = orig_point.x - minMaxX.first->x;
        orig_point.y = orig_point.y - minMaxY.first->y;
        Point scaled_point(
            orig_point.x/scaleFactor,
            orig_point.y/scaleFactor
        );

        scaled.push_back(scaled_point);
    }

    return true;
}

extern "C" void app_main(void)
{
    camera_config_t config;
    camera_fb_t *fb = NULL;
    int i = 0;
    user_data_t userdata;
    std::vector<Point> centers;
    std::vector<Rect> rects;
    std::vector<Point> shrink_points;

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_CIF;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 24;
    config.fb_count = 2;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }
    sensor_t * s = esp_camera_sensor_get();
    ESP_LOGI(TAG, "Past camera init, PID is 0x%0X", s->id.PID);
    // s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_special_effect(s, 1);

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();
    int sock = do_connect();

    out = (uint8_t *)heap_caps_malloc(OUT_BUFLEN, (MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM));

    while(1) {
        uint64_t fr_start = esp_timer_get_time();
        fb = esp_camera_fb_get();

        jpeg.openRAM((uint8_t *)fb->buf, fb->len, drawMCUs);
        jpeg.setPixelType(EIGHT_BIT_GRAYSCALE);
        userdata.width = jpeg.getWidth();
        userdata.height = jpeg.getHeight();
        userdata.zero_points.clear();
        jpeg.setUserPointer(&userdata);
        jpeg.decode(0,0,0);


        findBlobRects(userdata.zero_points, rects);
        centers.clear();
        for (auto rit = rects.begin(); rit != rects.end(); rit++) {
            centers.push_back(rit->center());
        }

        handle_incoming_frame(centers);
        uint64_t fr_end = esp_timer_get_time();
        jpeg.close();
        esp_camera_fb_return(fb);

        i++;

        size_t sendlen = 0;
        sendlen = snprintf((char *)out, OUT_BUFLEN, "[\"STAT\", %d, %d, %d]\n", filter_state, i, v);
        send(sock, out, sendlen, 0);

        sendlen = 0;
        sendlen = snprintf((char *)out+sendlen, OUT_BUFLEN-sendlen, "[\"WINNERS\", [");
        for (auto it = velocity_queue.begin(); it != velocity_queue.end(); it++) {
            if (it != velocity_queue.begin()) {
                sendlen += snprintf((char *)out+sendlen, OUT_BUFLEN-sendlen, ",");
            }
            sendlen += snprintf((char *)out+sendlen, OUT_BUFLEN-sendlen, "[%d, %d]", it->x, it->y);
        }
        sendlen += snprintf((char *)out+sendlen, OUT_BUFLEN-sendlen, "]]\n");
        send(sock, out, sendlen, 0);

        sendlen = 0;
        sendlen = snprintf((char *)out+sendlen, OUT_BUFLEN-sendlen, "[\"LOSERS\", [");
        for (auto it = loser_bin.bin.begin(); it != loser_bin.bin.end(); it++) {
            if (it != loser_bin.bin.begin()) {
                if (it->second < loser_bin.bin_threshold) {
                    continue;
                }
                sendlen += snprintf((char *)out+sendlen, OUT_BUFLEN-sendlen, ",");
            }
            sendlen += snprintf((char *)out+sendlen, OUT_BUFLEN-sendlen, "[%d, %d]", it->first.x, it->first.y);
        }
        sendlen += snprintf((char *)out+sendlen, OUT_BUFLEN-sendlen, "]]\n");
        send(sock, out, sendlen, 0);

        if (filter_state == COMMIT) {
            printf("Send\n");
            sendlen = 0;
            sendlen = snprintf((char *)out+sendlen, OUT_BUFLEN-sendlen, "[\"COMMIT\", [");
            shrink_points.clear();
            if (scale(path_point_queue, shrink_points)) {
                for (auto it = shrink_points.begin(); it != shrink_points.end(); it++) {
                    if (it != shrink_points.begin()) {
                        sendlen += snprintf((char *)out+sendlen, OUT_BUFLEN-sendlen, ",");
                    }
                    sendlen += snprintf((char *)out+sendlen, OUT_BUFLEN-sendlen, "[%d, %d]", it->x, it->y);
                }
            }

            filter_state = IDLE;
            detect_queue.clear();
            velocity_queue.clear();
            sendlen += snprintf((char *)out+sendlen, OUT_BUFLEN-sendlen, "]]\n");
            send(sock, out, sendlen, 0);
        }

        if (!(i%30)) {
            // heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);
            ESP_LOGI(
                TAG,
                "Time: %llu Frame: %d (%d) %d - (%d) - %d - %d (%d)",
                fr_end - fr_start,
                i,
                userdata.zero_points.capacity(),
                filter_state,
                centers.size(),
                loser_bin.bin.size(),
                loser_bin.prune(LOSER_BIN_LIFETIME_SECONDS),
                v
            );
        }
    }
}
