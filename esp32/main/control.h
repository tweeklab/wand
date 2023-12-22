#include "esp_event.h"

#include "freertos/message_buffer.h"

ESP_EVENT_DECLARE_BASE(WANDC_EVENT_BASE);
extern esp_event_loop_handle_t wandc_loop;

typedef struct {
    bool enable;
    unsigned int level;
} ir_control_t;

typedef enum {
    LIGHTSHOW_STATE_SLEEP_ENTER,
    LIGHTSHOW_STATE_SLEEP,
    LIGHTSHOW_STATE_SLEEP_EXIT,
    LIGHTSHOW_STATE_IDLE,
    LIGHTSHOW_STATE_IDLE_EXIT,
    LIGHTSHOW_STATE_TRAINING_ENTER,
    LIGHTSHOW_STATE_TRAINING,
    LIGHTSHOW_STATE_SPELL_FULFILL,
    LIGHTSHOW_STATE_SPELL_BLOCK
} lightshow_state_t;

typedef enum {
    WAND_EVENT_IDLE,
    WAND_EVENT_MOVEMENT,
    WAND_EVENT_SPELL_IGNORED,
    WAND_EVENT_SPELL_DETECT
} wand_event_kind_t;
typedef struct {
    wand_event_kind_t kind;
    uint16_t xy[2];
    char const *spell;
    uint8_t spell_prob;
} wand_event_t;

typedef enum {
    WANDC_EVENT_IP_STACK_READY,
    WANDC_EVENT_IR_ARRAY_CONFIG,
    WANDC_EVENT_WAND_EVENT,
    WANDC_SET_USER_ON,
    WANDC_SET_IR_LED_ON
} control_event_t;

void start_control_loop(void);