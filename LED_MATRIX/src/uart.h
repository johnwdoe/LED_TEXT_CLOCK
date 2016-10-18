/*
 * uart.h
 *
 *  Created on: May 27, 2016
 *      Author: william
 */

#ifndef UART_H_
#define UART_H_
#include <stdint.h>

#define BAUD_RATE (38400)

void uart_init(void);
void uart_send(uint8_t d);
extern void uart_onrecv(uint8_t d);

#endif /* UART_H_ */
