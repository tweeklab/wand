#include "esp_event.h"

#include "freertos/message_buffer.h"

ESP_EVENT_DECLARE_BASE(WANDC_EVENT_BASE);
extern esp_event_loop_handle_t wandc_loop;

typedef struct {
    MessageBufferHandle_t msgbuf;
} client_descriptor_t;

typedef struct {
    bool enable;
    unsigned int level;
} ir_control_t;

typedef enum {
    WANDC_EVENT_IP_STACK_READY,
    WANDC_EVENT_WEB_CLIENT_CONNECTED,
    WANDC_EVENT_WEB_CLIENT_DISCONNECTED,
    WANDC_EVENT_IR_ARRAY_CONFIG
} control_event_t;

void start_control_loop(void);