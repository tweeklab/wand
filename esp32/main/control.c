#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif_sntp.h"
#include "esp_http_client.h"
#include "freertos/message_buffer.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "control.h"
#include "detector.h"
#include "ui_server.h"

#include <sys/socket.h>
#include <netdb.h>
#include <time.h>

#define NUM_IDLE_STATES 4
#define TIME_PER_IDLE_STATE_SECONDS 120
#define CMD_BUF_BYTES 256

#define GLOBE_MCAST_ADDR 0
#define WREATH_MCAST_ADDR 1
static char *mcast_addrs[] = {
    "239.1.1.1",
    "239.1.1.2"
};
#define GLOBE_MCAST_MASK (1<<GLOBE_MCAST_ADDR)
#define WREATH_MCAST_MASK (1<<WREATH_MCAST_ADDR)

static const char *TAG = "wand control";
static bool has_ip = false;
static int mcast_control_sock = -1;
static TaskHandle_t heartbeat_task_handle = NULL;
static lightshow_state_t lightshow_state = LIGHTSHOW_STATE_IDLE;
static lightshow_state_t lightshow_exit_state = LIGHTSHOW_STATE_IDLE;
static char const *pending_spell;
static TaskHandle_t idle_lightshow_task_handle = NULL;
static SemaphoreHandle_t mcast_socket_mutex = NULL;
static unsigned int wand_active_counter = 0;
static int64_t last_active_timestamp = 0;

ESP_EVENT_DEFINE_BASE(WANDC_EVENT_BASE);
esp_event_loop_handle_t wandc_loop;

extern const char ifttt_tls_root_pem_start[] asm("_binary_ifttt_tls_root_pem_start");

static void init_mcast_control_sock() {
    mcast_control_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (mcast_control_sock < 0) {
        ESP_LOGE(TAG, "Failed to create mcast control. Error %d", errno);
        return;
    }
}

static int64_t get_time_us() {
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    return (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
}

static void send_multicast(int addr_mask, const char *message, bool duplicate) {
    struct sockaddr_in dest_addr;
    int err = 0;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(4545);

    if (mcast_control_sock < 0) {
        ESP_LOGI(TAG, "Create mcast control sockset.");
        init_mcast_control_sock();
    }

    for (int i=0; i<2; i++) {
        if (addr_mask & GLOBE_MCAST_MASK) {
            dest_addr.sin_addr.s_addr = inet_addr(mcast_addrs[GLOBE_MCAST_ADDR]);
            err = sendto(
                mcast_control_sock,
                message,
                strlen(message),
                0,
                (struct sockaddr *)&dest_addr,
                sizeof(dest_addr)
            );
        }
        if (addr_mask & WREATH_MCAST_MASK) {
            dest_addr.sin_addr.s_addr = inet_addr(mcast_addrs[WREATH_MCAST_ADDR]);
            err = sendto(
                mcast_control_sock,
                message,
                strlen(message),
                0,
                (struct sockaddr *)&dest_addr,
                sizeof(dest_addr)
            );
        }
        if (duplicate == false) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    }
}

static void display_blank(int addr_mask, char *cmd_buf) {
    snprintf(cmd_buf, CMD_BUF_BYTES, "{\"action\": \"abort\", \"blank\": true, \"id\": %llu}", get_time_us());
    send_multicast(addr_mask, cmd_buf, true);
}

static void display_global_fade(int addr_mask, char *cmd_buf) {
    snprintf(cmd_buf, CMD_BUF_BYTES, "{\"action\": \"effect\", \"args\": [\"global_fade_out\", 20], \"id\": %llu}", get_time_us());
    send_multicast(addr_mask, cmd_buf, true);
}

static void display_spell(int addr_mask, char *cmd_buf, const char *spell_name) {
    snprintf(cmd_buf, CMD_BUF_BYTES, "{\"action\": \"spell\", \"args\": [\"%s\"], \"id\": %llu}", spell_name, get_time_us());
    send_multicast(addr_mask, cmd_buf, true);
}

static void display_idle_state(int addr_mask, char *cmd_buf, const int idle_state_id) {
    snprintf(cmd_buf, CMD_BUF_BYTES, "{\"action\": \"idlestate\", \"args\": [%d], \"id\": %llu}", idle_state_id, get_time_us());
    send_multicast(addr_mask, cmd_buf, true);
}

static void set_wand_decorations_active(bool on) {
    esp_http_client_config_t config = {
        .method = HTTP_METHOD_GET,
        .cert_pem = ifttt_tls_root_pem_start
    };
    if (on) {
        config.url = "https://maker.ifttt.com/trigger/wand_active/json/with/key/VmD35TgKrycWxnjSSFJ2M";
    } else {
        config.url = "https://maker.ifttt.com/trigger/wand_inactive/json/with/key/VmD35TgKrycWxnjSSFJ2M";
    }
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_perform(client);
    esp_http_client_cleanup(client);
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

static void idle_lightshow() {
    int current_idle_state;
    int current_idle_state_time_counter;
    char *cmd_buf = (char *) heap_caps_malloc(
        CMD_BUF_BYTES,
        MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM
    );
    if (cmd_buf == NULL) {
        ESP_LOGE(TAG, "Unable to allocate command memory for idle_lightshow");
        goto out_memerror;
    }
    if (xSemaphoreTake(mcast_socket_mutex, 0) == pdFALSE) {
        // We expect if the idle lightshow was started, nothing else
        // is holding the mutex.  We'll hold the mutex here until we stop
        ESP_LOGE(TAG, "idle_lightshow could not get mutex!");
        goto out_semerror;
    }
    ESP_LOGI(TAG, "idle lightshow starting");
    current_idle_state = 0;
    current_idle_state_time_counter = 0;
    while (lightshow_state == LIGHTSHOW_STATE_IDLE) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        display_idle_state(GLOBE_MCAST_MASK|WREATH_MCAST_MASK, cmd_buf, current_idle_state);
        if (++current_idle_state_time_counter == TIME_PER_IDLE_STATE_SECONDS) {
            if (++current_idle_state == NUM_IDLE_STATES) {
                current_idle_state = 0;
            }
            current_idle_state_time_counter = 0;
        }
    }
    ESP_LOGI(TAG, "idle lightshow ending");

    xSemaphoreGive(mcast_socket_mutex);
out_semerror:
    heap_caps_free(cmd_buf);
out_memerror:
    idle_lightshow_task_handle = NULL;
    vTaskDelete(NULL);
}

static void start_idle_lightshow() {
    xTaskCreatePinnedToCore(
        idle_lightshow,
        "SHOW",
        3072,
        NULL,
        tskIDLE_PRIORITY,
        &idle_lightshow_task_handle,
        0
    );
}

static void handle_wand_event(void* handler_arg, esp_event_base_t base, int32_t id, void* data)
{
    char *cmd_buf = (char *)handler_arg;
    wand_event_t *event_data = (wand_event_t *)data;

    if (
        lightshow_state == LIGHTSHOW_STATE_SLEEP || 
        lightshow_state == LIGHTSHOW_STATE_SLEEP_ENTER
    ) {
        return;
    }

    if (event_data->kind == WAND_EVENT_MOVEMENT) {
        ++wand_active_counter;
        last_active_timestamp = get_time_us();
        if ((lightshow_state == LIGHTSHOW_STATE_IDLE) && (wand_active_counter > 20)) {
            lightshow_state = LIGHTSHOW_STATE_IDLE_EXIT;
            lightshow_exit_state = LIGHTSHOW_STATE_TRAINING_ENTER;
            xTaskNotifyGive(heartbeat_task_handle);
        }
        if (lightshow_state == LIGHTSHOW_STATE_TRAINING) {
            int16_t x = event_data->xy[0] - 320;
            int16_t y = -event_data->xy[1] + 240;
            snprintf(
                cmd_buf,
                CMD_BUF_BYTES,
                "{\"action\": \"effect\", \"args\": [\"wreath_render_wand_point\", %d, %d]}", x, y
            );
            send_multicast(WREATH_MCAST_MASK, cmd_buf, false);
        }
    } else if (event_data->kind == WAND_EVENT_IDLE) {
        if (wand_active_counter > 0) {
            --wand_active_counter;
        }
    } else if (event_data->kind == WAND_EVENT_SPELL_DETECT) {
        if (lightshow_state == LIGHTSHOW_STATE_SPELL_BLOCK) {
            return;
        }
        if (event_data->spell_prob < 230) {
            return;
        }
        pending_spell = event_data->spell;
        if (!strncmp(pending_spell, "discard", 7)) {
            return;
        }
        if (lightshow_state == LIGHTSHOW_STATE_IDLE) {
            lightshow_state = LIGHTSHOW_STATE_IDLE_EXIT;
            lightshow_exit_state = LIGHTSHOW_STATE_SPELL_FULFILL;
            xTaskNotifyGive(heartbeat_task_handle);
        } else {
            lightshow_state = LIGHTSHOW_STATE_SPELL_FULFILL;
            xTaskNotifyGive(heartbeat_task_handle);
        }
    } else {
        ESP_LOGI(TAG, "Other wand event: %d", event_data->kind);
    }
}

static void heartbeat() {
    struct tm now_tm;
    int64_t now = 0;
    time_t now_time_t = 0;
    int64_t spell_block_start_time = 0;
    lightshow_state = LIGHTSHOW_STATE_SLEEP_ENTER;
    char *cmd_buf = (char *) heap_caps_malloc(
        CMD_BUF_BYTES,
        MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM
    );
    while(1) {
        // Need to sleep with the possibility of waking up sooner
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

        now = get_time_us();
        now_time_t = now/1000000;
        localtime_r(&now_time_t, &now_tm);
        int linear_time = (now_tm.tm_hour*60) + now_tm.tm_min;

        if (
            ((linear_time >= (16*60)+0)) &&
            ((linear_time < (23*60)+03))
        ) {
            if (lightshow_state == LIGHTSHOW_STATE_SLEEP) {
                lightshow_state = LIGHTSHOW_STATE_SLEEP_EXIT;
            }
        } else if (
            lightshow_state != LIGHTSHOW_STATE_SLEEP && 
            lightshow_state != LIGHTSHOW_STATE_SLEEP_ENTER
        ) {
            lightshow_state = LIGHTSHOW_STATE_IDLE_EXIT;
            lightshow_exit_state = LIGHTSHOW_STATE_SLEEP_ENTER;
        }
        
        // Respond to wand idle
        if ((now - last_active_timestamp) > 5000000) {
            wand_active_counter = 0;
            if (lightshow_state == LIGHTSHOW_STATE_TRAINING) {
                lightshow_state = LIGHTSHOW_STATE_IDLE;
                ESP_LOGI(TAG, "Training session ends");
                display_global_fade(GLOBE_MCAST_MASK|WREATH_MCAST_MASK, cmd_buf);
            }
            if (lightshow_state == LIGHTSHOW_STATE_SPELL_WAIT) {
                lightshow_state = LIGHTSHOW_STATE_IDLE;
                display_global_fade(GLOBE_MCAST_MASK|WREATH_MCAST_MASK, cmd_buf);
            }
        }

        // Enforce current state
        if (lightshow_state == LIGHTSHOW_STATE_IDLE) {
            if (idle_lightshow_task_handle == NULL) {
                start_idle_lightshow();
            }
        } else if (lightshow_state == LIGHTSHOW_STATE_IDLE_EXIT) {
            ESP_LOGI(TAG, "Attempt to gracefully exit idle state.");
            if (idle_lightshow_task_handle != NULL) {
                xTaskNotifyGive(idle_lightshow_task_handle);
            }
            if (xSemaphoreTake(mcast_socket_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                xSemaphoreGive(mcast_socket_mutex);
                lightshow_state = lightshow_exit_state;
                xTaskNotifyGive(heartbeat_task_handle);
            }
        } else if (lightshow_state == LIGHTSHOW_STATE_TRAINING_ENTER) {
            ESP_LOGI(TAG, "Training session begins");
            lightshow_state = LIGHTSHOW_STATE_TRAINING;
            display_global_fade(GLOBE_MCAST_MASK, cmd_buf);
        }  else if (lightshow_state == LIGHTSHOW_STATE_SPELL_BLOCK) {
            if ((now - spell_block_start_time) > 5000000) {
                lightshow_state = LIGHTSHOW_STATE_SPELL_WAIT;
            }
        }  else if (lightshow_state == LIGHTSHOW_STATE_SPELL_FULFILL) {
            display_spell(GLOBE_MCAST_MASK|WREATH_MCAST_MASK, cmd_buf, pending_spell);
            lightshow_state = LIGHTSHOW_STATE_SPELL_BLOCK;
            spell_block_start_time = now;
        }  else if (lightshow_state == LIGHTSHOW_STATE_SLEEP_ENTER) {
            display_global_fade(GLOBE_MCAST_MASK|WREATH_MCAST_MASK, cmd_buf);
            set_wand_decorations_active(false);
            lightshow_state = LIGHTSHOW_STATE_SLEEP;
        } else if (lightshow_state == LIGHTSHOW_STATE_SLEEP) {
            display_blank(GLOBE_MCAST_MASK|WREATH_MCAST_MASK, cmd_buf);
        } else if (lightshow_state == LIGHTSHOW_STATE_SLEEP_EXIT) {
            set_wand_decorations_active(true);
            lightshow_state = LIGHTSHOW_STATE_IDLE;
        }
    }
}

static void start_heartbeat_task() {
    xTaskCreatePinnedToCore(
        heartbeat,
        "HB",
        3072,
        NULL,
        tskIDLE_PRIORITY,
        &heartbeat_task_handle,
        0
    );
}

static void handle_ip_stack_ready(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data)
{
    if (!has_ip) {
        ESP_LOGI(TAG, "IP stack is up");

        start_time_sync();
        start_heartbeat_task();
        start_ui_server();
        start_detector_tasks();
        has_ip = true;
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

    mcast_socket_mutex = xSemaphoreCreateMutex();

    esp_event_handler_register_with(wandc_loop, WANDC_EVENT_BASE, WANDC_EVENT_IP_STACK_READY, handle_ip_stack_ready, NULL);
    char *wand_status_cmd_buf = (char *) heap_caps_malloc(
        CMD_BUF_BYTES,
        MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM
    );
    esp_event_handler_register_with(wandc_loop, WANDC_EVENT_BASE, WANDC_EVENT_WAND_EVENT, handle_wand_event, wand_status_cmd_buf);
}
