#include <util/twi.h>

void TWI_init (uint8_t)

uint8_t TWI_start(uint8_t twi_addr, uint8_t read_write);

void TWI_stop(void);

uint8_t TWI_write(uint8_t tx_data); //write byte to the started device

uint8_t TWI_ack_read(void); // continuous read

uint8_t TWI_nack_read(void); //read and stop condition

uint8_t Read_Reg(uint8_t TWI_addr, uint8_t reg_addr);

uint8_t Read_Reg_N(uint8_t TWI_addr, uint8_t reg_addr, uint8_t bytes, int16_t data);

uint8_t Write_Reg(uint8_t TWI_addr, uint8_t reg_addr, uint8_t value);
