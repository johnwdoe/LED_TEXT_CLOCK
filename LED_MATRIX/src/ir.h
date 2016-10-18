/*
 * ir.h
 *
 *  Created on: May 27, 2016
 *      Author: william
 */

#ifndef IR_H_
#define IR_H_
#include <stdint.h>

#define IR_ADDR_H (0x02)
#define IR_ADDR_L (0xFF)

void ir_init(void);
extern void ir_oncmdrecv(uint8_t cmd);

#endif /* IR_H_ */
