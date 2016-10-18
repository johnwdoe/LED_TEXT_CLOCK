/*
 * i2c.c
 *
 *  Created on: Jun 29, 2014
 *      Author: william
 */

#include <avr/io.h>
#include <string.h>
#include "i2c.h"

void i2c_init(void){
	//set clock rate
	TWBR = _TWBR;
	TWSR |= (_TWPS << TWPS0);
}

uint8_t i2c_io (uint8_t dev_addr, uint8_t *idata, uint8_t idata_len, uint8_t *odata, uint8_t odata_len){
	uint8_t data_cursor;
	//Send start
	TWCR = (1<<TWINT | 1<<TWSTA | 1<<TWEN);
	while (!(TWCR & (1<<TWINT))); //wait for completion
	if((TWSR & 0xF8) != 0x08/*START*/) return R_I2C_ERR;
	//sending slave address with write flag + W
	TWDR = (dev_addr << 1);
	TWCR = (1<<TWINT | 1<<TWEN);
	while (!(TWCR & (1<<TWINT))); //wait
	if((TWSR & 0xF8) != 0x18/*SLA+W ACK*/) return R_I2C_ERR;
	//sending data bytes
	for(data_cursor=0; data_cursor<idata_len; data_cursor++){
		TWDR = *(idata+data_cursor);
		TWCR = (1<<TWINT | 1<<TWEN);
		while (!(TWCR & (1<<TWINT))); //wait
		if((TWSR & 0xF8) != 0x28/*DTA ACK*/) return R_I2C_ERR;
		//uart_send('i');
	}
	if (odata_len > 0){
		/*receive output data part*/
		//send stop & start
		TWCR = (1<<TWINT | 1<<TWSTA | 1<<TWEN | 1<<TWSTO);
		while (!(TWCR & (1<<TWINT))); //wait for completion
		if((TWSR & 0xF8) != 0x08/*START*/) return R_I2C_ERR;
		//send slave address + R
		TWDR = (dev_addr << 1 | 0x01);
		TWCR = (1<<TWINT | 1<<TWEN);
		while (!(TWCR & (1<<TWINT))); //wait
		if((TWSR & 0xF8) != 0x40/*SLA+R ACK*/) return R_I2C_ERR;
		//now i2c logic is in receive mode
		for(data_cursor = 0; data_cursor < odata_len; data_cursor++){
			//readout data
			if(data_cursor+1 == odata_len){
				//receive the last byte
				TWCR = (1<<TWINT | 1<<TWEN);
				while (!(TWCR & (1<<TWINT))); //wait
				if ((TWSR & 0xF8) != 0x58 /*DATA NACK*/) return R_I2C_ERR;
			}else{
				//receive byte
				TWCR = (1<<TWINT | 1<<TWEA | 1<<TWEN);
				while (!(TWCR & (1<<TWINT))); //wait
				if ((TWSR & 0xF8) != 0x50 /*DATA ACK*/) return R_I2C_ERR;
			}
			*(odata+data_cursor) = TWDR;
		}
	}
	//send stop
	TWCR = (1<<TWINT | 1<<TWEN | 1<<TWSTO);
	return R_I2C_OK;
}

uint8_t i2c_reg_write (uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t data_len){
	uint8_t tmp_buf[10];
	memcpy(tmp_buf+1, data, data_len);
	*tmp_buf = reg_addr; //reg address must be on the first position of data array
	return i2c_io(dev_addr, tmp_buf, data_len+1, NULL, 0);
}

uint8_t i2c_reg_read (uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t data_len){
	return i2c_io(dev_addr, &reg_addr, 1, data, data_len);
}


