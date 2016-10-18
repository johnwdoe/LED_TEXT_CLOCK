/*
 * mxcontrol.c
 *
 *  Created on: May 4, 2016
 *      Author: william
 */
#include "mxcontrol.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>


#define F_MX_BLINK (1)
#define F_MX_AUTOBRIGHT (2)

#define CUR_P_X ((printCoords>>4))
#define CUR_P_Y ((printCoords&0x0F))

uint8_t crow;
uint8_t spibuf[3];
uint8_t printCoords; //0xXY
volatile uint8_t vmem[9][2];
volatile uint8_t vbuf[9][2];
volatile uint8_t spiptr;
volatile uint8_t mx_flags = 0;
volatile uint8_t t0_ovf_cnt = 0;
extern uint8_t t0_extended_int;
volatile uint8_t autobrValue;

const PROGMEM uint8_t alpha[] = {
    0x35, 0b11111, 0b10001, 0b11111,    //0
    0x35, 0b10010, 0b11111, 0b10000,    //1
    0x35, 0b11101, 0b10101, 0b10111,    //2
    0x35, 0b10101, 0b10101, 0b11111,    //3
    0x35, 0b111, 0b100, 0b11111,    //4
    0x35, 0b10111, 0b10101, 0b11101,    //5
    0x35, 0b11111, 0b10101, 0b11101,    //6
    0x35, 0b1, 0b1, 0b11111,    //7
    0x35, 0b11111, 0b10101, 0b11111,    //8
    0x35, 0b10111, 0b10101, 0b11111,    //9

    0x35, 0b11110, 0b101, 0b11110,    //A
    0x35, 0b11111, 0b10101, 0b1010,    //B
    0x35, 0b1110, 0b10001, 0b1010,    //C
    0x35, 0b11111, 0b10001, 0b1110,    //D
    0x35, 0b11111, 0b10101, 0b10001,    //E
    0x35, 0b11111, 0b101, 0b1,    //F
    0x35, 0b1110, 0b10001, 0b11010,    //G
    0x35, 0b11111, 0b100, 0b11111,    //H
    0x15, 0b11111,    //I
    0x35, 0b1000, 0b10000, 0b1111,    //J
    0x35, 0b11111, 0b100, 0b11011,    //K
    0x35, 0b11111, 0b10000, 0b10000,    //L
    0x55, 0b11111, 0b10, 0b100, 0b10, 0b11111,    //M
    0x45, 0b11111, 0b10, 0b100, 0b11111,    //N
    0x35, 0b1110, 0b10001, 0b1110,    //O
    0x35, 0b11111, 0b101, 0b10,    //P
    0x45, 0b1110, 0b10001, 0b11001, 0b11110,    //Q
    0x35, 0b11111, 0b101, 0b111010,    //R
    0x35, 0b10010, 0b10101, 0b1001,    //S
    0x35, 0b1, 0b11111, 0b1,    //T
    0x45, 0b1111, 0b10000, 0b10000, 0b1111,    //U
    0x35, 0b1111, 0b10000, 0b1111,    //V
    0x55, 0b1111, 0b10000, 0b1000, 0b10000, 0b1111,    //W
    0x35, 0b11011, 0b100, 0b11011,    //X
    0x35, 0b10011, 0b10100, 0b1111,    //Y
    0x35, 0b11001, 0b10101, 0b10011,    //Z
    0xD7, 0b1000, 0b1111111, 0x00, 0b11100, 0b1000, 0x00, 0b1000, 0x00, 0b1000, 0b1111111, 0x00, 0b11100, 0b1000    //BATT
};

void mxcontrol_init(void){
	//set control pins direction
	DISPLAY_DDR |= (1<<MOSI_P | 1<<SCK_P | 1<<STB_P | 1<<OE_P);
	//set initial state
	DISPLAY_PORT |= (1<<OE_P); //disable output and reset

	//configure spi: Master, LSB first, FCPU/128
	SPCR |= (1<<SPE | 1<<DORD | 1<<MSTR | 1<<SPR1 | 1<<SPR0 | 1<<SPIE);

	//configure dim timer (1):
	//inverting mode, fast 8-bit, f.div by 8
	TCCR1A |= (1<<COM1B1 | 1<<COM1B0 | 1<<WGM10);
	TCCR1B |= (1<<WGM12 | 1<<CS11);


	//configure scan timer
	//T0 with /64 overflow!!!

	TIMSK |= (1<<TOIE0); //overflow interrupt enable
	TCCR0 |= (1<<CS01 | 1<<CS00); //start divided by 64

	//ADC
	ADMUX |= (1<<REFS0 | 1<<ADLAR); //reference - AVCC, left adjusted result
	ADCSRA |= (1<<ADIE | 7<<ADPS0); //interrupt enabled, prescaler = 128

	crow = 0; //row's cursor on a first row
	printCoords = 0x00;
	sei();
}

void mxcontrol_setbrightness(uint8_t b){
	OCR1BL = b;
}

void mxcontrol_setbrightnessauto(uint8_t b){
	if (b){
		mx_flags |= F_MX_AUTOBRIGHT;
		autobrValue = OCR1BL;
	}else{
		mx_flags &= ~F_MX_AUTOBRIGHT;
		ADCSRA &= ~(1<<ADEN);
	}
}

void mxcontrol_makepoint(uint8_t x, uint8_t y, uint8_t color){
	if (x>12 || y>8) return;
	if (color){
		vbuf[y][x>>3] |= (0x80 >> (x & 0x07));
	}else
	{
		vbuf[y][x>>3] &= ~(0x80 >> (x & 0x07));
	}

}

void mxcontrol_makehline(uint8_t x, uint8_t y, uint8_t lenght, uint8_t color){
	while (lenght--){
		mxcontrol_makepoint(x++, y, color);
	}
}

void mxcontrol_clear(void){
	uint8_t i;
	for (i=0; i<18; i++){
		*(vbuf[0]+i) = 0x00;
	}
}

void mxcontrol_draw(uint8_t mode){
	uint8_t i, j;
	if (mode == 0){
		//refresh all data immediately
		for (i=0; i<18; i++) *(vmem[0]+i) = *(vbuf[0]+i); //просто перекидываем буфер
	}else if (mode == 1 || mode == 2 || mode == 3){
		//побитовое копирование в видеопамять. общее время ориентировочно 3 сек
		for (i=0; i<9; i++){//row
			for (j=0; j<13; j++){//col
				if (mode == 2 || mode == 3){
					if (mode == 2){
						vmem[i][j>>3] |= (0x80 >> (j & 0x07));//turn on bit
					}else{
						vmem[i][j>>3] ^= (0x80 >> (j & 0x07));//toggle bit
					}
					_delay_ms(20);
				}
				vmem[i][j>>3] &= ~(0x80 >> (j & 0x07)); //turn off bit
				vmem[i][j>>3] |= (vbuf[i][j>>3] & (0x80 >> (j & 0x07))); //copy bit from buffer
				if (mode == 1) _delay_ms(10);
			}
		}

	}
}

void mxcontrol_blink(uint8_t b){
	if (b){
		t0_ovf_cnt = 0;
		mx_flags |= F_MX_BLINK; //set blink flag
	}else{
		mx_flags &= ~F_MX_BLINK;
		TCCR1A |= (1<<COM1B1 | 1<<COM1B0); //enable pwm-mode
	}
}

void mxcontrol_setprintcoords(uint8_t x, uint8_t y){
	printCoords = (x<<4) | (y&0x0F);
}

void mxcontrol_print(uint8_t c){
	uint8_t sl, sh, x, y, bitcol;
	uint8_t *alphaPtr = alpha;
	//seek ptr
	while (c--){
		alphaPtr += (1+(pgm_read_byte(alphaPtr)>>4));
	}
	//теперь alphaPtr смотрит на наш символ
	sl = pgm_read_byte(alphaPtr) >> 4;
	sh = pgm_read_byte(alphaPtr++) & 0x0F;
	//alphaPtr++;
	for (x=0;x<sl;x++){
		bitcol = pgm_read_byte(alphaPtr+x);
		for (y=0; y<sh; y++){
			mxcontrol_makepoint(CUR_P_X+x, CUR_P_Y+y, (bitcol&0x01));
			bitcol >>= 1;
		}
	}
	//incrementing X
	printCoords += ((sl<<4)+0x10);
}

void mxcontrol_printchar (char c){
	//fix char
	if (c >= '0' && c <= '9'){
		c -= '0';
	}else if (c >= 'A' && c <= 'Z'){
		c = c - 'A' + 10;
	}else if (c >= 'a' && c <= 'z'){
		c = c - 'a' + 10;
	}
	mxcontrol_print((uint8_t)c);
}


ISR(TIMER0_OVF_vect){
	t0_extended_int |= ((~t0_ovf_cnt) & (++t0_ovf_cnt)); //выставляем только те биты, что изменились с 0 на 1 в текущем такте. это и будет флагом прерывания

	//int /256
	if (t0_extended_int & T0_OVF_F_256){
		t0_extended_int &= ~T0_OVF_F_256; //reset flag
		if (mx_flags & F_MX_BLINK) TCCR1A ^= (1<<COM1B1 | 1<<COM1B0); //toggle OC1B <-> PORT mode
	}

	if (t0_extended_int & T0_OVF_F_8){
		t0_extended_int &= ~T0_OVF_F_8;
		if (mx_flags & F_MX_AUTOBRIGHT) ADCSRA |= (1<<ADEN | 1<<ADSC); //start conversion
	}


	//high
	spibuf[2] = vmem[crow][0];
	//middle
	spibuf[1] = vmem[crow][1] & ~0x07;
	if (crow < 3){
		spibuf[1] |= (0x04>>crow);
	}
	//low
	if (crow >= 3){
		spibuf[0] = (0x80 >> (crow-3));
	}else{
		spibuf[0] = 0x00;
	}
	//display strobe disable
	DISPLAY_PORT &= ~(1<<STB_P);

	//initiate SPI send
	SPDR = spibuf[spiptr=0];

	//set next row
	crow++;
	if(crow > 8){
		crow = 0;
	}
}

ISR(SPI_STC_vect){
	//on SPI transfer complete
	if (spiptr < 2){
		//send next byte
		SPDR = spibuf[++spiptr];
	}else{
		//it was a last byte. strobe row
		DISPLAY_PORT |= (1<<STB_P);
	}
}

ISR(ADC_vect){
	//result in ADCH
	if (ADCH > autobrValue){
		autobrValue++;
	}else if (ADCH < autobrValue){
		autobrValue--;
	}
	mxcontrol_setbrightness(autobrValue);
}
