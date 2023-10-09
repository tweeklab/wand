#include <stdio.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <sys/param.h>

#include "freertos/task.h"
#include "freertos/message_buffer.h"

#include <esp_http_server.h>

#include "ui_server.h"
#include "ui_html.h"
#include "detector.h"

static const char *TAG = "wand_httpd";
static httpd_handle_t server;
static MessageBufferHandle_t ui_msgbuf_handle;

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_send(req, ui_index_main, ui_index_main_len);
    return ESP_OK;
}

static esp_err_t sparklines_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/javascript");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_send(req, ui_jquery_sparkline_js_gz, ui_jquery_sparkline_js_gz_len);
    return ESP_OK;
}

static esp_err_t control_handler(httpd_req_t *req)
{
    esp_err_t ret = ESP_OK;

    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "New control connection was opened");
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
        // if (!strncmp(buf, "reset", strlen("reset"))) {
        //     i = 0;
        // }
    } 
    return ret;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    char *status_data;
    esp_err_t ret = ESP_OK;

    status_data = (char *)heap_caps_malloc(STATUS_DATA_BUFLEN, (MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM));

    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "New stream connection was opened");
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    ws_pkt.final = true;
    ws_pkt.payload = (uint8_t *)status_data;

    while(true) {
        xMessageBufferReceive(
            ui_msgbuf_handle,
            status_data,
            STATUS_DATA_BUFLEN,
            portMAX_DELAY
        );
        
        ws_pkt.len = strlen(status_data);
        ret = httpd_ws_send_frame(req, &ws_pkt);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
            break;
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
static const httpd_uri_t sparkline_serve = {
        .uri        = "/jquery.sparkline.js",
        .method     = HTTP_GET,
        .handler    = sparklines_handler,
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


static const httpd_uri_t stream = {
        .uri        = "/ws",
        .method     = HTTP_GET,
        .handler    = stream_handler,
        .user_ctx   = NULL,
        .is_websocket = true
};


void start_ui_server(MessageBufferHandle_t msgbuf)
{
    httpd_handle_t control_server = NULL;
    httpd_handle_t stream_server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.core_id = 0;
    config.server_port = 3000;
    config.ctrl_port = 4000;

    ui_msgbuf_handle = msgbuf;

    ESP_LOGI(TAG, "Starting control server on port: '%d'", config.server_port);
    if (httpd_start(&control_server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering control URI handlers");
        httpd_register_uri_handler(control_server, &index_serve);
        httpd_register_uri_handler(control_server, &sparkline_serve);
        httpd_register_uri_handler(control_server, &control);
    }

    config.server_port += 1;
    config.ctrl_port += 1;
    ESP_LOGI(TAG, "Starting stream server on port: '%d'", config.server_port);
    if (httpd_start(&stream_server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering stream URI handler");
        httpd_register_uri_handler(stream_server, &stream);
    }
}

void stop_ui_server(void)
{
    // Stop the httpd server
    httpd_stop(server);
}
