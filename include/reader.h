#include <stdint.h>
#include <esp_err.h>

#define RC522_I2C_ADDR 0x28
#define RC522_REG_COMMAND 0x01
#define RC522_ComIEnReg 0x02
#define RC522_DivIEnReg 0x03
#define RC522_ComIrqReg 0x04
#define RC522_ModWidthReg 0x24
#define RC522_TimerModeReg 0x2A
#define RC522_TimerPrescalerReg 0x2B
#define RC522_TimerReloadMSBReg 0x2C
#define RC522_TimerReloadLSBReg 0x2D
#define RC522_TxControlReg 0x14
#define RC522_RfCfgReg 0x26
#define RC522_TxAskReg 0x15
#define RC522ModeReg 0x11
#define RC522_VERSION_REG 0x37



// Initialize the RC522 reader
void reader_init(void(*cb)(void*arg));

// Read the UID of a card
void reader_read_uid(char* uid);

// Read data from block
void reader_read_block(char* data, int block);

// Read data from sector
void reader_read_sector(char* data, int sector);

// Magic NFC card detection (1 = true, 0 = false)
uint8_t reader_is_magic_card();

// Interrupt Pin callback (when reader detects a card)
void reader_irq_handler(void(*cb)(void*arg));

// Access Raw Register Read
esp_err_t reader_read_reg_read(uint8_t reg, uint8_t *data);

// Access Raw Register Write
esp_err_t reader_write_reg_read(uint8_t reg, uint8_t data);