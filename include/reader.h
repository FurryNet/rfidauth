#include <stdint.h>

#define RC522_I2C_ADDR 0x28
#define RC522_REG_COMMAND 0x01
#define RC522_ComIEnReg 0x2
#define RC522_DivIEnReg 0x3




// Initialize the RC522 reader
void reader_init();

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