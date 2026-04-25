#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sensors.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_ERROR_CHECK(init_sensors());
    ESP_LOGI(TAG, "Sensor interrupt logging test running");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
