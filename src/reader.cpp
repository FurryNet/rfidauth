#include <reader.h>
#include <esp_log.h>
#include <display.h>
#include <driver/gpio.h>
#include <driver/i2c.h>

#define sda_pin 6
#define scl_pin 7
#define irq_pin GPIO_NUM_5 // Interrupt pin when card is detected
#define TAG "RC522"


esp_err_t reg_read(uint8_t reg, uint8_t *data);
esp_err_t reg_read(uint8_t reg, uint8_t *data, uint8_t len);
esp_err_t reg_write(uint8_t reg, uint8_t data);
esp_err_t reg_write(uint8_t reg, uint8_t *data, uint8_t len);

void reader_init() {
    ESP_LOGI(TAG, "Initializing IRQ Pin Service");
    // Setup GPIO service
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << irq_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
    gpio_install_isr_service(0);

    // Enable interrupt config on the reader
    reg_write(RC522_ComIEnReg, 0x80);
    reg_write(RC522_ComIEnReg, 0x7F);
    reg_write(RC522_DivIEnReg, 0x14);

    ESP_LOGI(TAG, "IRQ Setup Complete...");

    ESP_LOGI(TAG, "Initializing RC522 Reader...");
    
};


void reader_read_uid(char* uid) {

};


void reader_read_block(char* data, int block) {

};


void reader_read_sector(char* data, int sector) {

};


uint8_t reader_is_magic_card() {
    return 0;
};

void reader_irq_handler(void(*cb)(void*arg)) {
    gpio_isr_handler_add(irq_pin, cb, NULL);
}



// Internal Register Read/Write

esp_err_t reg_read(uint8_t reg, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RC522_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RC522_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);


    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read byte from register");
        return ret;
    }
    return ESP_OK;
}

// Write a byte to the specified register
esp_err_t reg_write(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (RC522_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write byte to register");
        return ret;
    }
    return ESP_OK;
}