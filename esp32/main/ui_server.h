#include <freertos/task.h>
#include <esp_http_server.h>

void start_ui_server();
void stop_ui_server(void);

typedef struct {
    httpd_handle_t handle;
    int fd;
} async_ws_stream_info_t;

typedef struct {
    async_ws_stream_info_t *stream_info;
    httpd_ws_frame_t *ws_pkt;
    TaskHandle_t sender_task;
    esp_err_t ret;
} async_packet_info_t;