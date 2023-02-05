#include <stdio.h>
#include <cstring>
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "img_converters.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "opencv_import.hpp"
#include "wifi.h"

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

typedef struct {
        uint16_t width;
        uint16_t height;
        uint16_t data_offset;
        const uint8_t *input;
        uint8_t *output;
} my_rgb_jpg_decoder;

//input buffer
static size_t  _my_jpg_read(void * arg, size_t index, uint8_t *buf, size_t len)
{
    my_rgb_jpg_decoder * jpeg = (my_rgb_jpg_decoder *)arg;
    if(buf) {
        memcpy(buf, jpeg->input + index, len);
    }
    return len;
}

static bool _my_rgb_write(void * arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data)
{
    my_rgb_jpg_decoder * jpeg = (my_rgb_jpg_decoder *)arg;
    if(!data){
        if(x == 0 && y == 0){
            //write start
            jpeg->width = w;
            jpeg->height = h;
        } else {
            //write end
        }
        return true;
    }

    size_t jw = jpeg->width;
    size_t t = y * jw;
    size_t b = t + (h * jw);
    size_t l = x;
    uint8_t *out = jpeg->output+jpeg->data_offset;
    uint8_t *o = out;
    size_t iy, ix;

    w = w;

    for(iy=t; iy<b; iy+=jw) {
        o = out+iy+l;
        for(ix=0; ix<w; ix+=1) {
            o[ix] = data[ix];
            // o[ix] = (data[ix+2] + data[ix+1] + data[ix])/3;
            // if (o[ix] > 100)
            //     o[ix] = 255;
            // else
            //     o[ix] = 0;
        }
        data+=w;
    }
    return true;
}


extern "C" void app_main(void)
{
    camera_config_t config;
    camera_fb_t *fb = NULL;
    int i = 0;

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
    config.frame_size = FRAMESIZE_QCIF;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 8;
    config.fb_count = 2;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }
    sensor_t * s = esp_camera_sensor_get();
    ESP_LOGI(TAG, "Past camera init, PID is 0x%0X", s->id.PID);

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    // out = (uint8_t*) heap_caps_malloc(176*144, (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    out = (uint8_t*) heap_caps_malloc(176*144, (MALLOC_CAP_8BIT));
    my_rgb_jpg_decoder jpeg;

    while(1) {
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        fb = esp_camera_fb_get();

        jpeg.width = 0;
        jpeg.height = 0;
        jpeg.input = fb->buf;
        jpeg.output = out;
        jpeg.data_offset = 0;

        uint64_t fr_start = esp_timer_get_time();
        if(esp_jpg_decode(fb->len, JPG_SCALE_NONE, _my_jpg_read, _my_rgb_write, (void*)&jpeg) != ESP_OK){
            ESP_LOGW(TAG, "jped decode fail");
        }
        opencv_func(out);
        i++;

        uint64_t fr_end = esp_timer_get_time();
        if (!(i%30))
            ESP_LOGI(TAG, "Time: %llu. %d (%d, %d)", fr_end - fr_start, fb->len, jpeg.width, jpeg.height);
        esp_camera_fb_return(fb);
    }
}