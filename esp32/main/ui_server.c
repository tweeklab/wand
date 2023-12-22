#include <stdio.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <sys/param.h>

#include "freertos/task.h"
#include "freertos/message_buffer.h"

#include <esp_http_server.h>
#include <esp_camera.h>

#include "ui_server.h"
#include "detector.h"
#include "control.h"

#define POINT_MSGBUF_SIZE (10*1024)

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

static const char *TAG = "wand_httpd";
static httpd_handle_t server;

static esp_err_t index_handler(httpd_req_t *req)
{
    extern const char index_html_start[] asm("_binary_index_html_start");
    extern const char index_html_end[]   asm("_binary_index_html_end");
    ssize_t index_html_len = index_html_end - index_html_start;
    httpd_resp_send(req, index_html_start, index_html_len);
    return ESP_OK;
}

static esp_err_t video_handler(httpd_req_t *req)
{
    esp_err_t ret = ESP_OK;
    char *part_buf[128];
    MessageBufferHandle_t video_msgbuf_handle;
    camera_fb_t recv_frame;

    ESP_LOGI(TAG, "Got video stream connection.");

    video_msgbuf_handle = xMessageBufferCreate(sizeof(camera_fb_t) + sizeof(size_t));

    ret = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (ret != ESP_OK)
    {
        return ret;
    }
    // httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    // httpd_resp_set_hdr(req, "X-Framerate", "60");

    set_detector_video_stream_buffer(video_msgbuf_handle);

    while(true) {
        xMessageBufferReceive(
            video_msgbuf_handle,
            &recv_frame,
            sizeof(camera_fb_t),
            portMAX_DELAY
        );

        if (ret == ESP_OK)
        {
            ret = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if (ret == ESP_OK)
        {
            size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, recv_frame.len, recv_frame.timestamp.tv_sec, recv_frame.timestamp.tv_usec);
            ret = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if (ret == ESP_OK)
        {
            ret = httpd_resp_send_chunk(req, (const char *)recv_frame.buf, recv_frame.len);
        }

        heap_caps_free(recv_frame.buf);
        
        if (ret != ESP_OK)
            break;
    }
    ESP_LOGI(TAG, "Stopping video stream");
    vMessageBufferDelete(video_msgbuf_handle);
    set_detector_video_stream_buffer(NULL);
    return ret;
}

static void async_send_ws_pkt(void *param) {
    esp_err_t ret = ESP_OK;
    async_packet_info_t *ws_packet_info = (async_packet_info_t *)param;
    ret = httpd_ws_send_frame_async(
        ws_packet_info->stream_info->handle,
        ws_packet_info->stream_info->fd,
        ws_packet_info->ws_pkt
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_send_frame_async failed with %d", ret);
    }
    ws_packet_info->ret = ret;
    xTaskNotifyGive(ws_packet_info->sender_task);
}

void point_stream_connection_task(void *params) {
    esp_err_t ret = ESP_OK;
    httpd_ws_frame_t ws_pkt;
    char *status_data;
    MessageBufferHandle_t point_msgbuf_handle;
    uint8_t *point_msgbuf_memory = NULL;
    StaticMessageBuffer_t point_msgbuf_struct;
    TaskHandle_t me = xTaskGetCurrentTaskHandle();
    async_ws_stream_info_t *stream_info = (async_ws_stream_info_t *)params;
    async_packet_info_t ws_packet_info = {
        .stream_info = stream_info,
        .ws_pkt = &ws_pkt,
        .sender_task = me,
        .ret = ESP_OK
    };

    ESP_LOGI(TAG, "Starting control stream");

    status_data = (char *)heap_caps_malloc(
        POINT_DATA_BUFLEN,
        (MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM)
    );
    point_msgbuf_memory = (uint8_t *) heap_caps_malloc(
        POINT_MSGBUF_SIZE,
        MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM
    );
    point_msgbuf_handle = xMessageBufferCreateStatic(
        POINT_MSGBUF_SIZE,
        point_msgbuf_memory,
        &point_msgbuf_struct
    );
    set_detector_point_stream_buffer(point_msgbuf_handle);


    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    ws_pkt.final = true;
    ws_pkt.payload = (uint8_t *)status_data;

    while(true) {
        xMessageBufferReceive(
            point_msgbuf_handle,
            status_data,
            POINT_DATA_BUFLEN,
            portMAX_DELAY
        );

        ws_pkt.len = strlen(status_data);
        ws_packet_info.ret = ESP_OK;
        if (httpd_queue_work(stream_info->handle, async_send_ws_pkt, &ws_packet_info) != ESP_OK) {
            ESP_LOGE(TAG, "httpd_queue_work failed with %d", ret);
            break;
        }
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (ws_packet_info.ret != ESP_OK) {
            // Error is reported in the other task
            break;
        }
    }

    ESP_LOGI(TAG, "Stopping control stream");
    // TODO: figure out of I really need to do this.  Assuming if we got
    // here right now that the client is gone
    httpd_sess_trigger_close(stream_info->handle, stream_info->fd);
    set_detector_point_stream_buffer(NULL);
    vMessageBufferDelete(point_msgbuf_handle);
    heap_caps_free(status_data);
    heap_caps_free(point_msgbuf_memory);
    heap_caps_free(stream_info);
    vTaskDelete(NULL);
}

static esp_err_t control_handler(httpd_req_t *req)
{
    BaseType_t task_result;
    esp_err_t ret = ESP_OK;
    async_ws_stream_info_t *stream_info;

    stream_info = (async_ws_stream_info_t *)heap_caps_malloc(
        sizeof(async_ws_stream_info_t),
        (MALLOC_CAP_8BIT)
    );

    if (req->method == HTTP_GET) {
        stream_info->handle = req->handle;
        stream_info->fd =  httpd_req_to_sockfd(req);
        task_result = xTaskCreatePinnedToCore(
            point_stream_connection_task,
            "point_stream",
            3000,
            stream_info,
            tskIDLE_PRIORITY,
            NULL,
            0
        );
        if (task_result != pdTRUE) {
            ESP_LOGE(TAG, "Failed to create point streamer task");
            free(stream_info);
        }
        ESP_LOGI(TAG, "Control ws negotiation complete");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Got control message!");

    httpd_ws_frame_t ws_pkt;
    char *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    /* Set max_len = 0 to get the frame len */
    ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);
    if (ws_pkt.len) {
        /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = (uint8_t *)buf;
        /* Set max_len = ws_pkt.len to get the frame payload */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);

        if (!strncmp((const char *)ws_pkt.payload, "show_on", strlen("show_on"))) {
            bool on = true;
            esp_event_post_to(
                wandc_loop,
                WANDC_EVENT_BASE,
                WANDC_SET_USER_ON,
                &on,
                sizeof(bool),
                portTICK_PERIOD_MS
            );
        } else if (!strncmp((const char *)ws_pkt.payload, "show_off", strlen("show_off"))) {
            bool on = false;
            esp_event_post_to(
                wandc_loop,
                WANDC_EVENT_BASE,
                WANDC_SET_USER_ON,
                &on,
                sizeof(bool),
                portTICK_PERIOD_MS
            );
        } else if (!strncmp((const char *)ws_pkt.payload, "ir_led_on", strlen("ir_led_on"))) {
            bool on = true;
            esp_event_post_to(
                wandc_loop,
                WANDC_EVENT_BASE,
                WANDC_SET_IR_LED_ON,
                &on,
                sizeof(bool),
                portTICK_PERIOD_MS
            );
        } else if (!strncmp((const char *)ws_pkt.payload, "ir_led_off", strlen("ir_led_off"))) {
            bool on = false;
            esp_event_post_to(
                wandc_loop,
                WANDC_EVENT_BASE,
                WANDC_SET_IR_LED_ON,
                &on,
                sizeof(bool),
                portTICK_PERIOD_MS
            );
        } else {
            ESP_LOGI(TAG, "Unhandled command: %s", ws_pkt.payload);
        }

    } 
    return ret;
}

static const httpd_uri_t index_serve = {
        .uri        = "/",
        .method     = HTTP_GET,
        .handler    = index_handler,
        .user_ctx   = NULL,
        .is_websocket = false
};

static const httpd_uri_t control = {
        .uri        = "/control",
        .method     = HTTP_GET,
        .handler    = control_handler,
        .user_ctx   = NULL,
        .is_websocket = true
};

static const httpd_uri_t video_stream = {
        .uri        = "/video_stream",
        .method     = HTTP_GET,
        .handler    = video_handler,
        .user_ctx   = NULL,
        .is_websocket = true
};


void start_ui_server(void)
{
    httpd_handle_t main_server = NULL;
    httpd_handle_t video_stream_server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.enable_so_linger = true;
    config.linger_timeout = 0;
    config.lru_purge_enable = true;
    config.core_id = 0;
    config.server_port = 80;
    config.ctrl_port = 3000;

    ESP_LOGI(TAG, "Starting main server on port: '%d'", config.server_port);
    if (httpd_start(&main_server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering main URI handlers");
        httpd_register_uri_handler(main_server, &index_serve);
        httpd_register_uri_handler(main_server, &control);
    }

    config.server_port = 8080;
    config.ctrl_port = 3001;
    ESP_LOGI(TAG, "Starting video stream server on port: '%d'", config.server_port);
    if (httpd_start(&video_stream_server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering video stream URI handler");
        httpd_register_uri_handler(video_stream_server, &video_stream);
    }
}

void stop_ui_server(void)
{
    // Stop the httpd server
    httpd_stop(server);
}
