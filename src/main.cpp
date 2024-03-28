#include <display.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <reader.h>
#include "driver/spi_master.h"

#define sda_pin 6
#define scl_pin 7
#define frequency 400000

void i2c_setup();

void cb_func(void*arg) {
    display_text("Card Detected!");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    display_clear();
}

extern "C" {
    void app_main() {
        ESP_LOGI("SYS", "Loading Components...");
        i2c_setup();
        display_init();
        ESP_LOGI("SYS", "Starting Script...");
        display_text("Hello, World!");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        display_clear();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        display_text("Goodbye, World!");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        display_clear();
        reader_init();
        reader_irq_handler(cb_func);
        while(true) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

// Initialize the I2C bus
void i2c_setup() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = frequency;
    conf.clk_flags = 0;
    i2c_param_config(I2C_NUM_0, &conf);
    if(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0) == ESP_OK)
        ESP_LOGI("I2C_Setup", "hdc2080 driver installed successfully");
    else
        ESP_LOGE("I2C_Setup", "hdc2080 driver failed to install");
}