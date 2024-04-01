#include <reader.h>
#include <esp_log.h>
#include <display.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <driver/spi_master.h>
#include <string.h>

#define sda_pin 6
#define scl_pin 7
#define cs_pin 4
#define irq_pin GPIO_NUM_5 // Interrupt pin when card is detected
#define TAG "RC522"


esp_err_t reg_read(uint8_t reg, uint8_t *data);
esp_err_t reg_read(uint8_t reg, uint8_t *data, uint8_t len);
esp_err_t reg_write(uint8_t reg, uint8_t data);
esp_err_t reg_write(uint8_t reg, uint8_t *data, uint8_t len);

spi_device_handle_t spi_handle;

esp_err_t  rc522_set_bitmask(uint8_t addr, uint8_t mask)
{
    uint8_t tmp;
    reg_read(addr, &tmp);
    return reg_write(addr, tmp | mask);
}


void reader_init() {
    // Setup Device interface
    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 1000000,
        .spics_io_num = cs_pin,
        .queue_size = 7,
    };
    if(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle) != ESP_OK)
        ESP_LOGI(TAG, "Failed to add spi bus device");

    // First startup initalization
    esp_err_t err = ESP_OK;
    uint8_t tmp = 0;

    const uint8_t test_addr = RC522_ModWidthReg, test_val = 0x25;
        uint8_t pass = 0;

        for(uint8_t i = test_val; i < test_val + 2; i++) {
            err = reg_write(test_addr, i);

            if(err == ESP_OK) {
                err = reg_read(test_addr, &tmp);

                if(err == ESP_OK && tmp == i) {
                    pass = 1;
                }
            }

            if(pass != 1) {
                ESP_LOGE(TAG, "Read/write test failed");
                return;
            }
        }
        // ------- End of RW test --------

        reg_write(RC522_REG_COMMAND, 0x0F);
        reg_write(RC522_TimerModeReg, 0x8D);
        reg_write(RC522_TimerPrescalerReg, 0x3E);
        reg_write(RC522_TimerReloadMSBReg, 0x1E);
        reg_write(RC522_TimerReloadLSBReg, 0x00);
        reg_write(RC522_TxAskReg, 0x40);
        reg_write(RC522ModeReg, 0x3D);

        // Enable the antenna
        reg_read(RC522_TxControlReg, &tmp);
        if(~ (tmp & 0x03))
            rc522_set_bitmask(RC522_TxControlReg, 0x03);
        reg_write(RC522_RfCfgReg, 0x60); // 43dB gain

        // Get the firmware version
        reg_read(RC522_VERSION_REG, &tmp);
        ESP_LOGI(TAG, "Initialized (firmware v%d.0)", (tmp & 0x03));


    // Setup GPIO service
    ESP_LOGI(TAG, "Initializing IRQ Pin Service");
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << irq_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);
    gpio_install_isr_service(0);


    // Enable interrupt config on the reader
    reg_write(RC522_ComIrqReg, 0x80);
    reg_write(RC522_ComIEnReg, 0x7F);
    reg_write(RC522_DivIEnReg, 0x14);
    ESP_LOGI(TAG, "IRQ Setup Complete...");
    
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

// Alias for global usage
esp_err_t reader_read_reg_read(uint8_t reg, uint8_t *data) {
    return reg_read(reg, data);
}
esp_err_t reader_write_reg_read(uint8_t reg, uint8_t data) {
    return reg_write(reg, data);
}


// Internal functions (Referenced: https://github.com/abobija/esp-idf-rc522/blob/main/src/rc522.c)

esp_err_t reg_read(uint8_t reg, uint8_t *data) {
    return reg_read(reg, data, 1);
}
esp_err_t reg_read(uint8_t reg, uint8_t *data, uint8_t len) {
    /* Full-Duplex Mode*/
    esp_err_t err = ESP_OK;

    // Write Pipeline
    spi_transaction_t write_tx;
    memset(&write_tx, 0, sizeof(spi_transaction_t));
    write_tx.flags = SPI_TRANS_USE_TXDATA;
    write_tx.length = 8;
    write_tx.tx_data[0] = ((reg << 1) & 0x7E) | 0x80;

    err = spi_device_transmit(spi_handle, &write_tx);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Read Mode: Failed to write to SPI device");
        return err;
    }

    // Read Pipeline
    spi_transaction_t read_tx;
    memset(&read_tx, 0, sizeof(spi_transaction_t));

    read_tx.flags = 0x0;
    read_tx.length = 8;
    read_tx.rxlength = 8 * len;
    read_tx.rx_buffer = data;

    err = spi_device_transmit(spi_handle, &read_tx);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Read Mode: Failed to read from SPI device");
        return err;
    }

    return err;
}

esp_err_t reg_write(uint8_t reg, uint8_t data) {
    return reg_write(reg, &data, 1);
}

esp_err_t reg_write(uint8_t reg, uint8_t *data, uint8_t len) {
    spi_transaction_t transaction;
    uint8_t buffer[len + 1];

    memset(&transaction, 0, sizeof(spi_transaction_t));
    transaction.length = 8 * (len + 1);
    transaction.tx_buffer = buffer;
    buffer[0] = (reg << 1) & 0x7E;
    memcpy(&buffer[1], data, len);

    return spi_device_transmit(spi_handle, &transaction);
}