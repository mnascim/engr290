#include <util/twi.h>

#define SCL_CLOCK 100000L

volatile struct {
	uint8_t TX_new_data:1;
	uint8_t TX_finished:1;
	uint8_t TX_buffer1_empty:1;
	uint8_t TX_buffer2_empty:1;
	uint8_t RX_flag:3;
	uint8_t TWI_ACK:1;
} flags;

volatile uint8_t TWI_status, TWI_byte;

//=============================== TWI functions ======================
void TWI_init (){
  // TWI init
  TWSR=0; // no prescaler
  TWBR=(uint8_t)(((F_CPU/SCL_CLOCK)-16)>>1);  //setting SCL; must be >10 for stable operation
 
}

uint8_t TWI_start(uint8_t twi_addr, uint8_t read_write) {

	TWCR=((1<<TWINT)|(1<<TWSTA)|(1<<TWEN)); //send START condition
	while(!(TWCR&(1<<TWINT))); //wait until transmission completed
	if (((TWSR&0xF8)!=TW_START)&&((TWSR&0xF8)!=TW_REP_START)) return 1; //something went wrong

	twi_addr=((twi_addr<<1)|(read_write&1)); //setting r/w bit
	TWDR=twi_addr; //send device address
	TWCR=((1<<TWINT)|(1<<TWEN)); //reset the flag

	while(!(TWCR&(1<<TWINT))); // wail until transmission completed and ACK/NACK has been received
	if (((TWSR&0xF8)!=TW_MT_SLA_ACK)&&((TWSR&0xF8)!=TW_MR_SLA_ACK)) return 2;
	return 0;
}

void inline TWI_stop(void) {
	TWCR=((1<<TWINT)|(1<<TWEN)|(1<<TWSTO)); // send stop condition
	while(TWCR&(1<<TWSTO)); // wait until stop condition is executed and bus released
}

uint8_t TWI_write(uint8_t tx_data) //write byte to the started device
{	
	TWDR=tx_data;
	TWCR=((1<<TWINT)|(1<<TWEN));
	while(!(TWCR & (1<<TWINT))); // wait until transmission completed
	if((TWSR&0xF8)!=TW_MT_DATA_ACK) return 1; //check value of TWI Status Register. Mask prescaler bits. Write failed
	return 0;
}

uint8_t TWI_ack_read(void) // continuous read
{
	TWCR=((1<<TWINT)|(1<<TWEN)|(1<<TWEA)); // Start read cycle
	while(!(TWCR&(1<<TWINT)));
	flags.TWI_ACK=1;    
	if((TWSR&0xF8)!=TW_MR_DATA_ACK) flags.TWI_ACK=0; //check value of TWI Status Register. Mask prescaler bits. Read failed
    return TWDR;
}


uint8_t TWI_nack_read(void) //read and stop condition
{
	TWCR=((1<<TWINT)|(1<<TWEN));
	while(!(TWCR&(1<<TWINT)));
	flags.TWI_ACK=1;    
	if((TWSR&0xF8)!=TW_MR_DATA_NACK) return flags.TWI_ACK=0; //check value of TWI Status Register. Mask prescaler bits. Read failed
    return TWDR;
}


uint8_t Read_Reg(uint8_t TWI_addr, uint8_t reg_addr){

	TWI_status=TWI_start (TWI_addr, TW_WRITE);
	if (TWI_status) return 1;

	TWI_status=TWI_write (reg_addr); //  register #
	if (TWI_status) return 2;

	TWI_status=TWI_start (TWI_addr, TW_READ);
	if (TWI_status) return 3;

	TWI_byte=TWI_nack_read();
	TWI_stop();
	if (!flags.TWI_ACK) return 4;
	return 0;
}


uint8_t Read_Reg_N(uint8_t TWI_addr, uint8_t reg_addr, uint8_t bytes, int16_t* data){ 
	
	uint8_t *p_data=(uint8_t*)data;

	TWI_status=TWI_start (TWI_addr, TW_WRITE);
	if (TWI_status) return 1;

	TWI_status=TWI_write (reg_addr); //  register #
	if (TWI_status) return 2;

	TWI_status=TWI_start (TWI_addr, TW_READ);
	if (TWI_status) return 3;
//	p_data=(uint8_t*)data;	
	for (uint8_t i=0; i<bytes-1; i++) {
		*p_data=TWI_ack_read();
		if (!flags.TWI_ACK) return 5;
		p_data++;	
	}
	*p_data=TWI_nack_read();
	TWI_stop();
	if (!flags.TWI_ACK) return 4;
	return 0;
}


uint8_t Write_Reg(uint8_t TWI_addr, uint8_t reg_addr, uint8_t value){

	TWI_status=TWI_start (TWI_addr, TW_WRITE);
	if (TWI_status) return 1;

	TWI_status=TWI_write (reg_addr); //  register #
	if (TWI_status) return 2;

	TWI_status=TWI_write (value); // write the value
	if (TWI_status) return 3;

	TWI_stop();
	if (!flags.TWI_ACK) return 4;

	return 0;
}

//=============================== TWI functions end ===================
