#include <stdio.h>
#include <cstring>
#include <vector>
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "opencv_import.hpp"
#include "wifi.h"
#include "JPEGDEC.h"
#include "blob_rect.hpp"
#include <sys/socket.h>
#include <netdb.h>            // struct addrinfo

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

#define OUT_BUFLEN 1024

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

    int top = pDraw->y * user->height;
    int bottom = top + (pDraw->iHeight * user->height);

    for (int y = top; y < bottom; y+=user->height) {
        for (int x = 0; x < pDraw->iWidth; x++) {
            pixel = (pIn[x] > 30 ? 255 : 0);
            if (pixel == 0) {
                user->zero_points.push_back(Point(pDraw->x + x, y/user->height));
            }
        }
        pIn += pDraw->iWidth;
    }
    return 1;
}

extern "C" void app_main(void)
{
    camera_config_t config;
    camera_fb_t *fb = NULL;
    int i = 0;
    user_data_t userdata;
    std::vector<Point> centers;
    std::vector<Rect> rects;

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

    out = (uint8_t *)heap_caps_malloc(OUT_BUFLEN, (MALLOC_CAP_8BIT));

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

        uint64_t fr_end = esp_timer_get_time();
        i++;

        findBlobRects(userdata.zero_points, rects);

        size_t sendlen;
        sendlen = snprintf((char *)out, OUT_BUFLEN, "[%d%s", i, rects.size() > 0 ? "," : "");
        for (size_t i = 0; i < rects.size(); i++) {
            Point center = rects[i].center();
            sendlen += snprintf((char *)out+sendlen, OUT_BUFLEN-sendlen, "[%d, %d]", center.x, center.y);
            if (i < rects.size() - 1) {
                sendlen += snprintf((char *)out+sendlen, OUT_BUFLEN-sendlen, ",");
            }
        }
        sendlen += snprintf((char *)out+sendlen, OUT_BUFLEN-sendlen, "]\n");

        ssize_t r = send(sock, out, sendlen, 0);
        if (!(i%30)) {
            ESP_LOGI(TAG, "Time: %llu. %d (%d, %d - %d - %d) - %d", fr_end - fr_start, fb->len, jpeg.getWidth(), jpeg.getHeight(), userdata.zero_points.size(), userdata.zero_points.capacity(), r);
        }
        jpeg.close();
        esp_camera_fb_return(fb);
    }
}