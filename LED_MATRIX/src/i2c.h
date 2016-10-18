/*
 * i2c.h
 *
 *  Created on: Jun 28, 2014
 *      Author: william
 */

#ifndef I2C_H_
#define I2C_H_
#include <stdint.h>

//settings for i2c speed (333.(3) kHz)
#define _TWPS (0)
#define _TWBR (4)

#define R_I2C_OK (0)
#define R_I2C_ERR (0xFF)

void i2c_init(void);
uint8_t i2c_io (uint8_t dev_addr, uint8_t *idata, uint8_t idata_len, uint8_t *odata, uint8_t odata_len);
uint8_t i2c_reg_write (uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t data_len);
uint8_t i2c_reg_read (uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t data_len);

#endif /* I2C_H_ */
