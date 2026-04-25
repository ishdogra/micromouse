#include "sensors.h"

#include <math.h>
#include <inttypes.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"

// IR Sensor definitions
#define OBSTACLE_SENSOR_FORWARD 26
#define REPEAT_DELAY_US 500000ull // 500 ms

// IMU Sensor definition
#define I2C_MASTER_SCL_IO             20       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO             22       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM                I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ            100000  /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE     0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE     0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS         1000

#define MPU6050_SENSOR_ADDR           0x68        /*!< Address of the MPU6050 sensor */
#define MPU6050_WHO_AM_I_REG_ADDR     0x75        /*!< Register addresses of the "who am I" register */
#define MPU6050_PWR_MGMT_1_REG_ADDR   0x6B        /*!< Register addresses of the power management register */
#define MPU6050_ACCEL_CONFIG_REG_ADDR 0x1C
#define ACCEL_ZOUT                    0x3F        /*!< Register addresses of the accelerometer z register */
#define MPU6050_WHO_AM_I_EXPECTED     0x70
#define MPU6050_PWR_WAKE_VALUE        0x00
#define MPU6050_ACCEL_FS_SEL_2G       0x00

#define MPU6050_SENSOR_GPIO           25

// Sensor interrupt and task definitions
#define SENSOR_QUEUE_LENGTH           10
#define SENSOR_TASK_STACK_SIZE        2048
#define IMU_POLL_PERIOD_MS            100

static const char *TAG = "sensors";
static volatile uint64_t last_log_time_us = 0;
static const float z_accel = 9.0f;
static QueueHandle_t sensor_queue;

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

/**
 * @brief Read a sequence of bytes from a MPU6050 sensor registers
 */
static esp_err_t mpu6050_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t mpu6050_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t mpu6050_init_device(void)
{
    uint8_t who_am_i = 0;
    esp_err_t err = mpu6050_register_read(dev_handle, MPU6050_WHO_AM_I_REG_ADDR, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "MPU6050 WHO_AM_I = 0x%02X", who_am_i);
    if (who_am_i != MPU6050_WHO_AM_I_EXPECTED) {
        ESP_LOGE(TAG, "Unexpected MPU6050 WHO_AM_I value");
        return ESP_ERR_INVALID_RESPONSE;
    }

    err = mpu6050_register_write_byte(dev_handle, MPU6050_PWR_MGMT_1_REG_ADDR, MPU6050_PWR_WAKE_VALUE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake MPU6050: %s", esp_err_to_name(err));
        return err;
    }

    err = mpu6050_register_write_byte(dev_handle, MPU6050_ACCEL_CONFIG_REG_ADDR, MPU6050_ACCEL_FS_SEL_2G);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accel range: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

static float read_z_accel_over_i2c(void) 
{
    uint8_t accel_bytes[2] = {0};
    esp_err_t err = mpu6050_register_read(dev_handle, ACCEL_ZOUT, accel_bytes, sizeof(accel_bytes));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accel Z registers: %s", esp_err_to_name(err));
        return NAN;
    }

    int16_t raw_z = (int16_t)((accel_bytes[0] << 8) | accel_bytes[1]);

    float accel_z_g = raw_z / 16384.0f;   
    float accel_z_ms2 = accel_z_g * 9.81f;
    
    return accel_z_ms2;
}

static void ir_helper(uint32_t gpio_num) 
{
    const uint64_t now = esp_timer_get_time();
    if (now - last_log_time_us >= REPEAT_DELAY_US) {
        ESP_LOGI(TAG, "Obstacle sensed on GPIO %" PRIu32, gpio_num);
        last_log_time_us = now;
    }
}

static void imu_helper(void) 
{
    float curr_z_accel = read_z_accel_over_i2c();
    if (isnan(curr_z_accel)) {
        return;
    }

    ESP_LOGI(TAG, "Accel Z = %.2f m/s^2", curr_z_accel);

    if (curr_z_accel < z_accel) {
        ESP_LOGI(TAG, "Robot is no longer on horizontal plane");
    }
}

static void IRAM_ATTR sensor_interrupt_handler(void *args)
{
    const uint32_t gpio_num = (uint32_t)(uintptr_t)args;
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (sensor_queue != NULL) {
        xQueueSendFromISR(sensor_queue, &gpio_num, &higher_priority_task_woken);
    }

    if (higher_priority_task_woken) {
        portYIELD_FROM_ISR();
    }
}

static void sensor_task(void *args)
{
    (void)args;
    uint32_t gpio_num;

    while (true) {
        if (xQueueReceive(sensor_queue, &gpio_num, pdMS_TO_TICKS(IMU_POLL_PERIOD_MS)) == pdTRUE) {
            switch(gpio_num) {
                case(OBSTACLE_SENSOR_FORWARD):
                    ir_helper(gpio_num);
                    break;
                default:
                    ESP_LOGW(TAG, "Unhandled sensor interrupt on GPIO %" PRIu32, gpio_num);
                    break;
            }
        }

        imu_helper();
    }
}


/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

esp_err_t init_sensors(void)
{
    if (sensor_queue != NULL) {
        return ESP_OK;
    }

    sensor_queue = xQueueCreate(SENSOR_QUEUE_LENGTH, sizeof(uint32_t));
    if (sensor_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor queue");
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(sensor_task, "sensor_task",
                    SENSOR_TASK_STACK_SIZE, NULL, 10, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
        vQueueDelete(sensor_queue);
        sensor_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    gpio_config_t forward_sensor_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << OBSTACLE_SENSOR_FORWARD),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };

    esp_err_t err = gpio_config(&forward_sensor_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure forward sensor GPIO: %s", esp_err_to_name(err));
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(err));
        return err;
    }

    err = gpio_isr_handler_add(OBSTACLE_SENSOR_FORWARD,
                               sensor_interrupt_handler,
                               (void *)(uintptr_t)OBSTACLE_SENSOR_FORWARD);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add GPIO ISR handler: %s", esp_err_to_name(err));
        return err;
    }

    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    err = mpu6050_init_device();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Sensor interrupt test armed on GPIO %d", OBSTACLE_SENSOR_FORWARD);
    return ESP_OK;
}
