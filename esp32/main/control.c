#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif_sntp.h"
#include "freertos/message_buffer.h"
#include "freertos/task.h"

#include "control.h"
#include "detector.h"
#include "ui_server.h"

#include <sys/socket.h>
#include <netdb.h>
#include <time.h>

#define CMD_BUF_BYTES 1024

static const char *TAG = "wand control";
static bool has_ip = false;
static int mcast_control_sock = -1;
static TaskHandle_t wake_status_task_handle = NULL;

ESP_EVENT_DEFINE_BASE(WANDC_EVENT_BASE);
esp_event_loop_handle_t wandc_loop;

static void init_mcast_control_sock() {
    mcast_control_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (mcast_control_sock < 0) {
        ESP_LOGE(TAG, "Failed to create mcast control. Error %d", errno);
        return;
    }
}

static void send_multicast(const char *message) {
    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_addr.s_addr = inet_addr("239.1.1.1");
    dest_addr.sin_port = htons(4545);

    if (mcast_control_sock < 0) {
        ESP_LOGI(TAG, "Create mcast control sockset.");
        init_mcast_control_sock();
    }

    int err = sendto(
        mcast_control_sock,
        message,
        strlen(message),
        0,
        (struct sockaddr *)&dest_addr,
        sizeof(dest_addr)
    );
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    } else {
        ESP_LOGI(TAG, "Message sent");
    }
}

static void start_time_sync() {
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&config);
    time_t now;
    char strftime_buf[64];
    struct tm timeinfo;

    if (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update system time within 10s timeout");
    }

    time(&now);
    setenv("TZ", "EST5EDT", 1);
    tzset();

    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "Time set: %s", strftime_buf);
}

void wake_status_update() {
    while(1) {
        // ESP_LOGI(TAG, "Wake check");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void start_wake_status_task() {
    xTaskCreatePinnedToCore(
        wake_status_update,
        "WAKE",
        3072,
        NULL,
        tskIDLE_PRIORITY,
        &wake_status_task_handle,
        0
    );
}

static void handle_ip_stack_ready(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data)
{
    if (!has_ip) {
        ESP_LOGI(TAG, "IP stack is up");

        start_time_sync();
        start_wake_status_task();
        start_ui_server();
        start_detector_tasks();
        has_ip = true;
    } 
}

static int64_t get_time_us() {
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    return (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
}

static void handle_wand_status_change(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data)
{
    char *cmd_buf = (char *)handler_arg;
    wand_status_change_t *change_data = (wand_status_change_t *)event_data;
    if (change_data->status == WAND_STATUS_PATTERN_MATCH) {
        ESP_LOGI(TAG, "Spell has been cast: %s", change_data->pattern);
        if (!strncmp(change_data->pattern, "incendio", strlen(change_data->pattern))) {
            snprintf(cmd_buf, CMD_BUF_BYTES, "{\"action\": \"effect\", \"args\": [\"sparkle\"]}");
            send_multicast(cmd_buf);
            return;
        }
        if (!strncmp(change_data->pattern, "descendo", strlen(change_data->pattern))) {
            snprintf(cmd_buf, CMD_BUF_BYTES, "{\"action\": \"abort\", \"blank\": true}");
            send_multicast(cmd_buf);
            return;
        }
    }
    if (change_data->status == WAND_STATUS_WAKEUP) {
        snprintf(cmd_buf, CMD_BUF_BYTES, "{\"action\": \"effect\", \"args\": [\"fade\",0,0,0.6,40]}");
        send_multicast(cmd_buf);
    }
}


void start_control_loop(void)
{
    esp_event_loop_args_t loop_args = {
        .queue_size = 10,
        .task_name = "wandc_loop",
        .task_priority = tskIDLE_PRIORITY+1,
        .task_stack_size = 3072,
        .task_core_id = 0
    };

    esp_event_loop_create(&loop_args, &wandc_loop);

    esp_event_handler_register_with(wandc_loop, WANDC_EVENT_BASE, WANDC_EVENT_IP_STACK_READY, handle_ip_stack_ready, NULL);

    char *wand_status_cmd_buf = (char *) heap_caps_malloc(
        CMD_BUF_BYTES,
        MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM
    );
    esp_event_handler_register_with(wandc_loop, WANDC_EVENT_BASE, WANDC_EVENT_WAND_STATUS_CHANGE, handle_wand_status_change, wand_status_cmd_buf);
}
