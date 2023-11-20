#include "nvs_flash.h"
#include "esp_log.h"

#include "control.h"
#include "wifi.h"

static const char *TAG = "wand main";

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
    start_wifi();
    // clear_wifi_config();
}
