#include "esp_event.h"

#include "freertos/message_buffer.h"

ESP_EVENT_DECLARE_BASE(WANDC_EVENT_BASE);
extern esp_event_loop_handle_t wandc_loop;

typedef struct {
    bool enable;
    unsigned int level;
} ir_control_t;

typedef enum {
    WAND_STATUS_PATTERN_MATCH,
    WAND_STATUS_PATTERN_DISCARD,
    WAND_STATUS_WAKEUP,
    WAND_STATUS_READY
} wand_status_t;
typedef struct {
    wand_status_t status;
    const char *pattern;
} wand_status_change_t;

typedef enum {
    WANDC_EVENT_IP_STACK_READY,
    WANDC_EVENT_IR_ARRAY_CONFIG,
    WANDC_EVENT_WAND_STATUS_CHANGE
} control_event_t;

void start_control_loop(void);