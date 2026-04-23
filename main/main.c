#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sm.h"

#define CONFIG_EXAMPLE_UART_PORT_NUM 0
#define CONFIG_EXAMPLE_UART_BAUD_RATE 115200
#define CONFIG_EXAMPLE_TASK_STACK_SIZE 3072

#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (CONFIG_EXAMPLE_UART_PORT_NUM)
#define ECHO_UART_BAUD_RATE     (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)

// Insert e-stop interrupt:
static QueueHandle_t interputQueue;

#define BUF_SIZE (1024)

// Interrupt test stuff:

static void IRAM_ATTR estop_interrupt_handler(void *args)
{
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) "ESTOP TRIGGERED\n", strlen("ESTOP TRIGGERED\n"));
    // logic to stop motors at lowest level 
}

static QueueHandle_t sm_event_queue;

void app_main(void)
{
    sm_t mouse_sm;
    sm_init(&mouse_sm);

    sm_event_queue = xQueueCreate(10, sizeof(sm_event_t));

    if (sm_event_queue == NULL) {
        // Queue creation failed
        return;
    }

    while (true) {
        sm_event_t event;

        if (xQueueReceive(sm_event_queue, &event, 0) == pdTRUE) {
            sm_handle_event(&mouse_sm, event);
        }

        sm_run_current_state(&mouse_sm);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
