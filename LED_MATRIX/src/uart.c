/*
 * uart.c
 *
 *  Created on: May 27, 2016
 *      Author: william
 */


#include "uart.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#define _UBRR (F_CPU/16/BAUD_RATE-1)


void uart_init(void){
	uint16_t u = _UBRR;
	DDRD |= (1<<PD1);
	UBRRH = (uint8_t)(u>>8);
	UBRRL = (uint8_t)u;
	UCSRB |= (1<<TXEN | 1<<RXEN | 1<<RXCIE);
	UCSRC |= (1<<URSEL | 1<<UCSZ1 | 1<<UCSZ0);
	sei();
}

void uart_send(uint8_t d){
	while (!(UCSRA & (1<<UDRE)));
	UDR = d;
}

ISR(USART_RXC_vect){
	uart_onrecv(UDR);
}
