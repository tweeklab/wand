#include "esp_log.h"
#include "esp_event.h"
#include "freertos/message_buffer.h"
#include "freertos/task.h"

#include "control.h"
#include "detector.h"
#include "ui_server.h"

#define UI_MSGBUF_SIZE 10240

static const char *TAG = "wand control";
static MessageBufferHandle_t ui_msgbuf_handle;
static uint8_t *ui_msgbuf_memory = NULL;
static StaticMessageBuffer_t ui_msgbuf_struct;
static bool has_ip = false;

ESP_EVENT_DEFINE_BASE(WANDC_EVENT_BASE);
esp_event_loop_handle_t wandc_loop;

void handle_ip_stack_ready(void* handler_arg, esp_event_base_t base, int32_t id, void* event_data)
{
    if (!has_ip) {
        ESP_LOGI(TAG, "IP stack is up");
        ui_msgbuf_memory = (uint8_t *) heap_caps_malloc(
            UI_MSGBUF_SIZE,
            MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM
        );
        ui_msgbuf_handle = xMessageBufferCreateStatic(
            UI_MSGBUF_SIZE,
            ui_msgbuf_memory,
            &ui_msgbuf_struct
        );

        start_ui_server(ui_msgbuf_handle);
        start_detector_tasks(ui_msgbuf_handle);
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

    esp_event_handler_register_with(wandc_loop, WANDC_EVENT_BASE, WANDC_EVENT_IP_STACK_READY, handle_ip_stack_ready, NULL);
}
