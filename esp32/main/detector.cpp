#include <stdio.h>
#include <cstring>
#include <vector>
#include <deque>
#include <algorithm>
#include <utility>

#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_log.h"

#include <sys/socket.h>
#include <netdb.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/message_buffer.h"
#include "freertos/semphr.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "JPEGDEC.h"
#include "blob_rect.hpp"
#include "point.hpp"
#include "atan_lut.hpp"
#include "cos_lut.hpp"
#include "wifi.h"
#include "detector.h"
#include "control.h"
#include "model.hpp"

using namespace std;

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    40
#define XCLK_GPIO_NUM     14
#define SIOD_GPIO_NUM     38
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       47
#define Y8_GPIO_NUM       13
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       10
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    41
#define HREF_GPIO_NUM     48
#define PCLK_GPIO_NUM     11

static const char *TAG = "wand detector";

static TaskHandle_t inference_task_handle = NULL;
static TaskHandle_t camera_task_handle = NULL;

static MessageBufferHandle_t point_msgbuf_handle = NULL;
SemaphoreHandle_t point_stream_mutex = NULL;
static MessageBufferHandle_t video_msgbuf_handle = NULL;
SemaphoreHandle_t video_stream_mutex = NULL;

static uint8_t *image = NULL;
static size_t image_pred = 0;
static uint8_t image_pred_score = 0;
static JPEGDEC jpeg;

#define FINAL_IMAGE_SIZE 40*29

#define RAW_FRAMES_SIZE 50
#define FILTER_QUEUE_SIZE 3
#define VELOCITY_QUEUE_SIZE 5
#define FRAME_QUEUE_HW_MARK 2
#define IDLE_FRAME_COUNT 15
#define LOSER_BIN_BINSIZE 32
#define MIN_IMAGE_BIN_COVERAGE 5
#define LOSER_BIN_THRESHOLD 10
#define LOSER_BIN_LIFETIME_SECONDS 1

typedef enum {
    IDLE,
    ACTIVE,
    DWELL,
    COMMIT
} filter_state_t;

#define TENSOR_ARENA_BYTES 50*1024
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input_tensor = nullptr;
static uint8_t *tensor_arena;

vector<string> labels = {
  "arresto-momentum",
  "descendo",
  "incendio",
  "mimblewimble",
  "locomotor",
  "tarantallegra",
  "gonadium",
  "boofisium",
  "discard"
};

// Labels we send to the control loop for lighting
// control
vector<string> action_labels = {
  "discard",
  "discard",
  "incendio",
  "mimblewimble",
  "discard",
  "discard",
  "gonadium",
  "discard",
  "discard"
};

// 220, 64
std::vector<Rect> ignore_regions = {
    Rect(Point(0,0), Point(100, 300))
};

deque<vector<Point>> detect_queue;
int idle_counter;
int frame_counter;
filter_state_t filter_state;
PointBin loser_bin(LOSER_BIN_BINSIZE, LOSER_BIN_THRESHOLD);
PointBin reduce_bin(10, 0);
wand_event_t wand_event_data;

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

int compute_bin_depth(PointBin const& bin) {
    using pair_type = decltype(bin.bin_count)::value_type;

    // https://stackoverflow.com/a/9371137
    auto pr = std::max_element
    (
        std::begin(bin.bin_count), std::end(bin.bin_count),
        [] (const pair_type & p1, const pair_type & p2) {
            return p1.second < p2.second;
        }
    );
    return pr->second;
}

void filter() {
    int min_sum = INT32_MAX;
    int winner_sum = 0;
    vector<Point> winner_points;
    Point winner(-1, -1);
    int depth;
    int min_depth = INT32_MAX; 

    // Build test grid
    for (auto it1 = filter_queue[0].begin(); it1 != filter_queue[0].end(); it1++) {
        if (loser_bin.contains(*it1))
            continue;
        for (auto it2 = filter_queue[1].begin(); it2 != filter_queue[1].end(); it2++) {
            if (loser_bin.contains(*it2))
                continue;
            for (auto it3 = filter_queue[2].begin(); it3 != filter_queue[2].end(); it3++) {
                if (loser_bin.contains(*it3))
                    continue;
                // The code in this inner loop now runs for every possible point combination of:
                // 1. The previous selected point
                // 2. a combination of one point from each of the frames in the filter_queue
                //    (3 frames worth)
                // If the most recent frame is represented by k, then the points are:
                // This looks like: selected_point -> k-2 -> k-1 -> k
                // 4 points total per iteration

                // Load all of the points from this iteration into a PointBin using
                // the same bin size that we use for the loser bin.
                // After that we check the PointBin to find the bin point with the
                // most actual points in the iteration.  We call this the frame's point
                // depth and the higher the depth, the more likely the iteration contains
                // a combination we don't want because it means more of the points are in one
                // spot.  This can happen if you have a persistent point of light in the frame
                // that was not loser-binned.  In this case when the wand comes in, it's very
                // likely that the wand points wind up being losers, because they are new
                // and farther from the existing point.
                // We can't just blindly filter points that are persistent because it interferes
                // with the wand dwelling at the end of casting a spell which would look no
                // different.  The method here favors an initially moving wand, by the time
                // the wand stops, the true losers should have been filtered by then and so
                // the stationary wand will be the only unfiltered point and you can hover forever
                // since even though the point depth will be high, it will still be the lowest depth.
                // It's the idea of the relativity of this measure that makes it work.
                // PointBin depth_bin(LOSER_BIN_BINSIZE, 0);
                // depth_bin.add(path_point_queue.back());
                // depth_bin.add(*it1);
                // depth_bin.add(*it2);
                // depth_bin.add(*it3);
                // depth = compute_bin_depth(depth_bin);
                // min_depth = std::min(min_depth, depth);

                Point v0 = *it1 - path_point_queue.back();
                Point v1 = *it2 - *it1;
                Point v2 = *it3 - *it2;
                int angle1 = atan2_lut(det(v0, v1), dot(v0, v1)) * norm_lut(v0);
                int angle2 = atan2_lut(det(v1, v2), dot(v1, v2)) * norm_lut(v1);
                int sum = (abs(angle1) + abs(angle2));
                min_sum = std::min(min_sum, sum);
                // if (min_sum == sum && depth == min_depth) {
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
    static int cumulative_losers = 0;

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
        if (filter_state == IDLE) {
            // When idle, only save enough frames to be able to compute
            // velocity.  Save a bit extra in case some frames are filtered
            // due to having no interesting points
            bounded_deque_push_back(detect_queue, tmp_pts, VELOCITY_QUEUE_SIZE*2);
        } else {
            bounded_deque_push_back(detect_queue, tmp_pts, RAW_FRAMES_SIZE);
        }
        idle_counter = 0;
    }

    if (idle_counter >= IDLE_FRAME_COUNT) {
        if (filter_state != IDLE) {
            wand_event_data.kind = WAND_EVENT_IDLE;
            esp_event_post_to(
                wandc_loop,
                WANDC_EVENT_BASE,
                WANDC_EVENT_WAND_EVENT,
                &wand_event_data,
                sizeof(wand_event_data),
                portTICK_PERIOD_MS
            );
        }
        filter_state = IDLE;
        detect_queue.clear();
        velocity_queue.clear();
        path_point_queue.clear();
    }

    if (idle_counter > 0)
        return;

    if ((++frame_counter % FRAME_QUEUE_HW_MARK) == 0) {
        path_point_queue.clear();
        filter_queue.clear();
        size_t before_bin_size = loser_bin.bin.size();
        for (auto fit = detect_queue.begin(); fit != detect_queue.end(); fit++) {
            process_single_frame(*fit);
        }
        size_t after_bin_size = loser_bin.bin.size();
        if ((after_bin_size - before_bin_size) > 0) {
            cumulative_losers += (after_bin_size - before_bin_size);
        } else {
            cumulative_losers = 0;
        }
        if (cumulative_losers > 5) {
            loser_bin.force_add(path_point_queue.back());
        }

        if (filter_state == ACTIVE) {
            wand_event_data.kind = WAND_EVENT_MOVEMENT;
            wand_event_data.xy[0] = velocity_queue.back().x;
            wand_event_data.xy[1] = velocity_queue.back().y;
            esp_event_post_to(
                wandc_loop,
                WANDC_EVENT_BASE,
                WANDC_EVENT_WAND_EVENT,
                &wand_event_data,
                sizeof(wand_event_data),
                portTICK_PERIOD_MS
            );
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
        // for (int x = 0; x < pDraw->iWidth; x++) {
        //     pixel = (pIn[x] > 2 ? 255 : 0);
        //     if (pixel == 0) {
        //         if (user->zero_points.size() < 512) {
        //             user->zero_points.push_back(Point(pDraw->x + x, y/user->width));
        //         }
        //     }
        // }
        for (int x = 0; x < pDraw->iWidth; x++) {
            pixel = (pIn[x] > 2 ? 255 : 0);
            if (pixel == 0) {
                if (user->zero_points.size() < 512) {
                    bool blocked = false;
                    Point newpt(pDraw->x + x, y/user->width);
                    for (auto it = ignore_regions.begin(); it != ignore_regions.end(); it++) {
                        if (it->contains(newpt)) {
                            blocked = true;
                        }
                    }
                    if (!blocked) {
                        user->zero_points.push_back(newpt);
                    }
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

    PointBin filter_bin(LOSER_BIN_BINSIZE, 0);

    int scaleFactor = 16;
    if ((ySize*100)/480 < (xSize*100)/640) {
        // Scale X axis
        scaleFactor = 1 + (16*xSize)/640;
    } else {
        // Scale Y axis
        scaleFactor = 1 + (16*ySize)/480;
    }

    for (auto it = orig.begin(); it != orig.end(); it++) {
        Point orig_point = *it;
        filter_bin.add(orig_point);
        orig_point.x = orig_point.x - minMaxX.first->x;
        orig_point.y = orig_point.y - minMaxY.first->y;
        Point scaled_point(
            orig_point.x/scaleFactor,
            orig_point.y/scaleFactor
        );

        scaled.push_back(scaled_point);
    }

    if (filter_bin.bin.size() > MIN_IMAGE_BIN_COVERAGE) {
        return true;
    }
    return false;
}

// Taken basically verbatim from: https://stackoverflow.com/a/14506390
void draw_line(uint8_t *buf, int x0, int y0, int x1, int y1) {
    int pixaddr = 0;
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sgnX = x0 < x1 ? 1 : -1;
    int sgnY = y0 < y1 ? 1 : -1;
    int e = 0;
    for (int i=0; i < dx+dy; i++) {
        pixaddr = x0 + (y0*40);
        if (pixaddr >= (40*29)) {
            ESP_LOGE(TAG, "pixaddr was: i=%d, x0=%d, y0=%d", pixaddr, x0, y0);
            continue;
        }
        buf[pixaddr] = 255;
        int e1 = e + dy;
        int e2 = e - dx;
        if (abs(e1) < abs(e2)) {
            x0 += sgnX;
            e = e1;
        } else {
            y0 += sgnY;
            e = e2;
        }
    }
}

extern "C" void inference_task(void *params)
{
    model = tflite::GetModel(model_tflite);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        MicroPrintf("Model provided is schema version %d not equal to supported "
                    "version %d.", model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }
    ESP_LOGI(TAG, "Model schema version: %lu", model->version());
    tensor_arena = (uint8_t *) heap_caps_malloc(
        TENSOR_ARENA_BYTES,
        MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM
    );
    if (tensor_arena == NULL) {
        ESP_LOGE(TAG, "Unable to allocate tensor_arena!");
        return;
    }

    static tflite::MicroMutableOpResolver<6> micro_op_resolver;
    micro_op_resolver.AddConv2D();
    micro_op_resolver.AddMaxPool2D();
    micro_op_resolver.AddQuantize();
    micro_op_resolver.AddReshape();
    micro_op_resolver.AddFullyConnected();
    micro_op_resolver.AddSoftmax();

    static tflite::MicroInterpreter static_interpreter(
        model, micro_op_resolver, tensor_arena, TENSOR_ARENA_BYTES
    );
    interpreter = &static_interpreter;
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        MicroPrintf("AllocateTensors() failed");
        return;
    }
    input_tensor = interpreter->input(0);
    MicroPrintf("Input tensor type: %d", input_tensor->type);
    MicroPrintf("Input tensor dims size: %d", input_tensor->dims->size);
    for (int i=0; i < input_tensor->dims->size; i++) {
        MicroPrintf("Input tensor dims[%d]: %d", i, input_tensor->dims->data[i]);
    }
    MicroPrintf("Input tensor bytes: %d", input_tensor->bytes);

    while(1) {
        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );
        memcpy(input_tensor->data.uint8, image, FINAL_IMAGE_SIZE);
        if (kTfLiteOk != interpreter->Invoke()) {
            ESP_LOGE(TAG, "Model invoke() failed.");
        }
        TfLiteTensor* output_tensor = interpreter->output(0);
        // Find the max index and map back to label
        size_t max_i = 0;
        uint8_t max_value = 0;
        for (size_t i = 0; i < output_tensor->bytes; i++) {
            uint8_t val = output_tensor->data.uint8[i];
            if (val > max_value) {
                max_value = val;
                max_i = i;
            }
        }
        image_pred = max_i;
        image_pred_score = max_value;
        xTaskNotifyGive(camera_task_handle);
    }
}

extern "C" void camera_task(void *params) {
    camera_config_t config;
    camera_fb_t *fb = NULL;
    int i = 0;
    char *status_data = NULL;
    size_t status_len = 0;
    uint8_t *buf_save;
    user_data_t userdata;
    std::vector<Point> centers;
    std::vector<Rect> rects;
    std::vector<Point> shrink_points;

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
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_VGA;
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

    // JSON messages sent to the web ui
    status_data = (char *)heap_caps_malloc(POINT_DATA_BUFLEN, (MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM));
    // Final drawn image shown to the model
    image = (uint8_t *)heap_caps_malloc(FINAL_IMAGE_SIZE, (MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM));

    while(1) {
        fb = esp_camera_fb_get();

        jpeg.openRAM((uint8_t *)fb->buf, fb->len, drawMCUs);
        jpeg.setPixelType(EIGHT_BIT_GRAYSCALE);
        userdata.width = jpeg.getWidth();
        userdata.height = jpeg.getHeight();
        userdata.zero_points.clear();
        jpeg.setUserPointer(&userdata);
        jpeg.decode(0,0,0);
        jpeg.close();

        findBlobRects(userdata.zero_points, rects);
        centers.clear();
        for (auto rit = rects.begin(); rit != rects.end(); rit++) {
            centers.push_back(rit->center());
        }

        handle_incoming_frame(centers);
        loser_bin.prune(LOSER_BIN_LIFETIME_SECONDS);
        
        if (xSemaphoreTake(video_stream_mutex, (100/portTICK_PERIOD_MS)) == pdTRUE) {
            if (video_msgbuf_handle && xMessageBufferIsEmpty(video_msgbuf_handle)) {
                buf_save = fb->buf;
                fb->buf = (uint8_t *)heap_caps_malloc(
                    fb->len,
                    (MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM)
                );
                memcpy(fb->buf, buf_save, fb->len);
                if (xMessageBufferSend(video_msgbuf_handle, fb, sizeof(camera_fb_t), 0) == 0) {
                    heap_caps_free(fb->buf);
                    ESP_LOGW(TAG, "Unable to send frame");
                }
                fb->buf = buf_save;
            }
            xSemaphoreGive(video_stream_mutex);
        }
        esp_camera_fb_return(fb);

        i++;

        if (xSemaphoreTake(point_stream_mutex, (100/portTICK_PERIOD_MS)) == pdTRUE) {
            if (point_msgbuf_handle) {
                status_len = 0;
                status_len = snprintf(
                    (char *)status_data, POINT_DATA_BUFLEN,
                    "{\"type\": \"status\", \"filter_state\": %d, \"frame\": %d, \"velocity\": %d}\n", filter_state, i, v
                );
                xMessageBufferSend(point_msgbuf_handle, status_data, status_len+1, 0);


                status_len = 0;
                status_len = snprintf((char *)status_data+status_len, POINT_DATA_BUFLEN-status_len,
                                        "{ \"type\": \"winners\", \"points\": [");
                for (auto it = path_point_queue.begin(); it != path_point_queue.end(); it++) {
                    if (it != path_point_queue.begin()) {
                        status_len += snprintf((char *)status_data+status_len, POINT_DATA_BUFLEN-status_len, ",");
                    }
                    status_len += snprintf((char *)status_data+status_len, POINT_DATA_BUFLEN-status_len, "[%d, %d]", it->x, it->y);
                }
                status_len += snprintf((char *)status_data+status_len, POINT_DATA_BUFLEN-status_len, "]}\n");
                xMessageBufferSend(point_msgbuf_handle, status_data, status_len+1, 0);


                status_len = 0;
                status_len = snprintf((char *)status_data+status_len, POINT_DATA_BUFLEN-status_len,
                                        "{ \"type\": \"centers\", \"points\": [");
                for (auto it = centers.begin(); it != centers.end(); it++) {
                    if (it != centers.begin()) {
                        status_len += snprintf((char *)status_data+status_len, POINT_DATA_BUFLEN-status_len, ",");
                    }
                    status_len += snprintf((char *)status_data+status_len, POINT_DATA_BUFLEN-status_len, "[%d, %d, %s]", it->x, it->y, (loser_bin.contains(*it) ? "true" : "false"));
                }
                status_len += snprintf((char *)status_data+status_len, POINT_DATA_BUFLEN-status_len, "]}\n");
                xMessageBufferSend(point_msgbuf_handle, status_data, status_len+1, 0);

                status_len = 0;
                status_len = snprintf((char *)status_data+status_len, POINT_DATA_BUFLEN-status_len,
                                        "{ \"type\": \"ignored\", \"rects\": [");
                for (auto it = ignore_regions.begin(); it != ignore_regions.end(); it++) {
                    if (it != ignore_regions.begin()) {
                        status_len += snprintf((char *)status_data+status_len, POINT_DATA_BUFLEN-status_len, ",");
                    }
                    status_len += snprintf((char *)status_data+status_len, POINT_DATA_BUFLEN-status_len, "[%d, %d, %d, %d]", it->tl.x, it->tl.y, it->br.x, it->br.y);
                }
                status_len += snprintf((char *)status_data+status_len, POINT_DATA_BUFLEN-status_len, "]}\n");
                xMessageBufferSend(point_msgbuf_handle, status_data, status_len+1, 0);
            }

            // This gets notified when an inference is made
            if (ulTaskNotifyTake(pdTRUE, 0)) {
                wand_event_data.kind = WAND_EVENT_SPELL_DETECT;
                wand_event_data.spell = action_labels[image_pred].c_str();
                wand_event_data.spell_prob = image_pred_score;
                esp_event_post_to(
                    wandc_loop,
                    WANDC_EVENT_BASE,
                    WANDC_EVENT_WAND_EVENT,
                    &wand_event_data,
                    sizeof(wand_event_data),
                    portTICK_PERIOD_MS
                );
                ESP_LOGI(TAG, "Inference result: %d -> %d (%s)", image_pred, image_pred_score, labels[image_pred].c_str());
                if (point_msgbuf_handle) {
                    snprintf(status_data, POINT_DATA_BUFLEN, "{\"type\": \"inference\", \"prob\": %d, \"label\": \"%s\"}", image_pred_score, labels[image_pred].c_str());
                    xMessageBufferSend(point_msgbuf_handle, status_data, strlen(status_data)+1, 0);
                }
            }
            xSemaphoreGive(point_stream_mutex);
        }

        // Detects when the wand hovers after drawing something
        // Triggers drawing the images from the path_point_queue and showing
        // it to the model.
        if (filter_state == COMMIT) {
            shrink_points.clear();
            memset(image, 0, FINAL_IMAGE_SIZE);
            if (scale(path_point_queue, shrink_points)) {
                Point prev_point(-1, -1);
                for (auto it = shrink_points.begin(); it != shrink_points.end(); it++) {
                    if (prev_point.x == -1) {
                        prev_point = *it;
                        continue;
                    }
                    draw_line(
                        image,
                        prev_point.x,
                        prev_point.y,
                        it->x,
                        it->y
                    );
                    prev_point = *it;
                }
                xTaskNotifyGive(inference_task_handle);
            } else {
                wand_event_data.kind = WAND_EVENT_SPELL_IGNORED;
                esp_event_post_to(
                    wandc_loop,
                    WANDC_EVENT_BASE,
                    WANDC_EVENT_WAND_EVENT,
                    &wand_event_data,
                    sizeof(wand_event_data),
                    portTICK_PERIOD_MS
                );
                ESP_LOGI(TAG, "Drop uninteresting image!");
            }

            filter_state = IDLE;
            detect_queue.clear();
            velocity_queue.clear();
        }
    }
}

    
void start_detector_tasks(void)
{
    point_stream_mutex = xSemaphoreCreateMutex();
    video_stream_mutex = xSemaphoreCreateMutex();

    if (inference_task_handle == NULL) {
        ESP_LOGI(TAG, "Starting inference task on CPU1");
        xTaskCreatePinnedToCore(
            inference_task,
            "INF",
            3072,
            NULL,
            tskIDLE_PRIORITY+18,
            &inference_task_handle,
            1
        );
    }

    if (camera_task_handle == NULL) {
        ESP_LOGI(TAG, "Starting camera task on CPU1");
        xTaskCreatePinnedToCore(
            camera_task,
            "CAM",
            15000,
            NULL,
            tskIDLE_PRIORITY+17,
            &camera_task_handle,
            1
        );
    }
}

void set_detector_point_stream_buffer(MessageBufferHandle_t msgbuf)
{
    if (!point_stream_mutex) {
        return;
    }
    if (xSemaphoreTake(point_stream_mutex, (500/portTICK_PERIOD_MS)) == pdTRUE) {
        point_msgbuf_handle = msgbuf;
        xSemaphoreGive(point_stream_mutex);
    } else {
        ESP_LOGW(TAG, "Timed out setting point_stream_buffer");
    }
}

void set_detector_video_stream_buffer(MessageBufferHandle_t msgbuf)
{
    if (!video_stream_mutex) {
        return;
    }
    if (xSemaphoreTake(video_stream_mutex, (500/portTICK_PERIOD_MS)) == pdTRUE) {
        video_msgbuf_handle = msgbuf;
        xSemaphoreGive(video_stream_mutex);
    } else {
        ESP_LOGW(TAG, "Timed out setting video_stream_buffer");
    }
}