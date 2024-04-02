#include <display.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <reader.h>
#include "driver/spi_master.h"
#include <string.h>

#define sda_pin 6
#define scl_pin 7
#define miso_pin 2
#define mosi_pin 10
#define sck_pin 11
#define frequency 400000

void i2c_setup();
void spi_setup();


void cb_func(void*arg) {
    display_write_page("Card Detected!", 3);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    display_clear();
}

extern "C" {
    void app_main() {
        ESP_LOGI("SYS", "Loading Components...");
        i2c_setup();
        display_init();
        spi_setup();
        reader_init(cb_func);

        ESP_LOGI("SYS", "Starting Script...");
        display_write_page("DEV Ver: ", 0);

        uint8_t verDat = 0;
        reader_read_reg_read(RC522_VERSION_REG, &verDat);
        uint8_t ver = (verDat & 0x03);
        uint8_t chipSet = (verDat >> 4) & 0x07;
        char verstr[4];
        char chipstr[4];
        sprintf(verstr, "%hhu", ver);
        sprintf(chipstr, "%hhu", chipSet);
        display_write_page(verstr, 1);
        display_write_page(chipstr, 2);
        
        
        while(1) {
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
        ESP_LOGI("I2C_Setup", "i2c driver installed successfully");
    else
        ESP_LOGE("I2C_Setup", "i2c driver failed to install");
}

// Initialize the SPI bus
spi_host_device_t spi_host;
void spi_setup() {
    spi_bus_config_t conf;
    memset(&conf, 0, sizeof(conf));
    conf.miso_io_num = miso_pin;
    conf.mosi_io_num = mosi_pin;
    conf.sclk_io_num = sck_pin;
    conf.quadwp_io_num = 20;
    conf.quadhd_io_num = 21;
    conf.flags = 0;
    conf.max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE;
    esp_err_t ret;
    if((ret = spi_bus_initialize(SPI2_HOST, &conf, SPI_DMA_DISABLED)) == ESP_OK)
        ESP_LOGI("SPI_Setup", "SPI bus initialized successfully");
    else
        ESP_LOGE("SPI_Setup", "SPI bus failed to initialize: %d", ret);
}