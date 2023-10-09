#include "nvs_flash.h"
#include "esp_console.h"
#include "esp_log.h"

#include "control.h"
#include "wifi.h"

static const char *TAG = "wand main";

// void start_console(void) {
//     esp_console_repl_t *repl = NULL;
//     esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
//     repl_config.task_priority = tskIDLE_PRIORITY+3;
//     /* Prompt to be printed before each line.
//      * This can be customized, made dynamic, etc.
//      */
//     repl_config.prompt = "wand>";
//     repl_config.max_cmdline_length = 30;
//     esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();

//     esp_console_register_help_command();
//     esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl);
//     ESP_ERROR_CHECK(esp_console_start_repl(repl));
// }

void app_main(void)
{
    ESP_LOGI(TAG, "app_main starting");

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ESP_ERROR_CHECK(nvs_flash_init());
    }

    start_control_loop();
    // start_console();
    start_wifi();
    // clear_wifi_config();
}
