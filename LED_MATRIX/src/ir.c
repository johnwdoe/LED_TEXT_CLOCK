/*
 * ir.c
 *
 *  Created on: May 27, 2016
 *      Author: william
 */

#include "ir.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define T2_START (TCCR2 |= (1<<CS22 | 1<<CS21 | 1<<CS20))
#define T2_STOP (TCCR2 &= ~(1<<CS22 | 1<<CS21 | 1<<CS20))
#define T2_PRESET (SFIOR |= (1<<PSR2))
//flags
#define F_START_RECEIVED	(1)
#define F_PACKET_RECEIVED	(2)
#define F_ERROR (4)

volatile uint8_t irflags;
volatile uint8_t packet[4];
volatile uint8_t bitptr;

void ir_init(void){
	//initialize interrupt 0
	MCUCR |= (1<<ISC01); //on falling edge
	GICR |= (1<<INT0);

	//initialize timer2. 1 tick = 128 us
	TCCR2 |= (1<<WGM21); //CTC
	OCR2 = 0x80; //16.4 ms
	TIMSK |= (1<<OCIE2); //output compare interrup enable

	irflags = 0x00;

	sei();
}

ISR(INT0_vect){
	uint8_t tick;
	tick = TCNT2; //store timervalue and restart counting
	T2_PRESET;
	TCNT2 = 0x00;
	T2_START;
	if(irflags & F_START_RECEIVED){
		//it can be 0 or 1 only!!!
		if(tick > 4 && tick < 23){
			//it's okay
			if(tick < 14){
				//logical 0
				packet[bitptr>>3] &= ~(1<<(bitptr & 0x07));
			}else{
				//logical 1
				packet[bitptr>>3] |= (1<<(bitptr & 0x07));
			}
			if (bitptr++ == 31){
				//packet received
				//check address and data
				if (packet[0] == IR_ADDR_H && packet[1] == IR_ADDR_L && (packet[2] ^ packet[3])==0xFF){
					//cmd in packet[2]. call extern
					ir_oncmdrecv(packet[2]);
					irflags = F_PACKET_RECEIVED;
				}else{
					irflags = F_ERROR;
				}
			}
		}else{
			irflags = F_ERROR;
		}
	}else{
		//it can be start only
		if (tick>82 && tick<128){
			//it's start condition
			irflags = F_START_RECEIVED;
			bitptr = 0x00;
		}else{
			irflags = F_ERROR;
		}
	}
}

ISR(TIMER2_COMP_vect){
	//incorrect command received
	T2_STOP; //stop timer
	T2_PRESET; //reset prescaler
	irflags = 0x00;
}
