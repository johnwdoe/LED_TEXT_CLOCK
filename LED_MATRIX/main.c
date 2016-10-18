/*
 * main.c
 *
 *  Created on: May 4, 2016
 *      Author: william
 */
#include "src/mxcontrol.h"
#include "src/ir.h"
#include "src/PCF2129.h"
#include "src/uart.h"
#include "src/i2c.h"
#include "src/bcd.h"
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>


#define F_IR_CMD_RECEIVED (1)
#define F_RTC_MINCHANGED (2)
#define F_RTC_LOWBATT (4)
#define F_UART_CMD_RECEIVED (8)

#define CMD_PWR (0x04)
#define CMD_TITLE (0x05)
#define CMD_UP (0x09)
#define CMD_DOWN (0x11)
#define CMD_LEFT (0x0C)
#define CMD_RIGHT (0x0E)
#define CMD_VOL_UP (0x50)
#define CMD_VOL_DOWN (0x54)
#define CMD_STOP (0x12)
#define CMD_PLAY (0x1C)
#define CMD_MODE_CH (0xFF) //TODO fix it!!!

//prototypes
void mapToMX(const uint8_t* map, uint8_t pos);
void timeToMX (uint8_t h, uint8_t m);
void ir_oncmdrecv(uint8_t cmd);
void uart_onrecv(uint8_t d);
void init(void);
void test(void);

const PROGMEM uint8_t hMap[] = {
		0x16, 0x08,	//TWELVE
		0x13, 0x04,	//ONE
		0x13, 0x44,	//TWO
		0x15, 0x84,	//THREE
		0x14, 0x05,	//FOUR
		0x14, 0x45,	//FIVE
		0x13, 0x06,	//SIX
		0x15, 0x85,	//SEVEN
		0x15, 0x36,	//EIGHT
		0x14, 0x96,	//NINE
		0x13, 0x07,	//TEN
		0x16, 0x37	//ELEVEN
};

const PROGMEM uint8_t mMap[] = {
		0x16, 0x78,										//[0] O'CLOCK
		0x34, 0x02, 0x07, 0x52,	0x04, 0x03,				//[5] FIVE MINUTES PAST
		0x33, 0xA0, 0x07, 0x52,	0x04, 0x03,				//[10] TEN MINUTES PAST
		0x27, 0x01,	0x04, 0x03,							//[15] QUARTER PAST
		0x36, 0x71, 0x07, 0x52,	0x04, 0x03,				//[20] TWENTY MINUTES PAST
		0x46, 0x71, 0x04, 0x02, 0x07, 0x52,	0x04, 0x03,	//[25] TWENTY FIVE MINUTES PAST
		0x24, 0x60, 0x04, 0x03,							//[30] HALF PAST
		0x46, 0x71, 0x04, 0x02, 0x07, 0x52, 0x02, 0x93,	//[35] TWENTY FIVE MINUTES TO
		0x36, 0x71, 0x07, 0x52, 0x02, 0x93,				//[40] TWENTY MINUTES TO
		0x27, 0x01, 0x02, 0x93,							//[45] QUARTER TO
		0x33, 0xA0, 0x07, 0x52, 0x02, 0x93,				//[50] TEN MINUTES TO
		0x34, 0x02, 0x07, 0x52, 0x02, 0x93				//[55] FIVE MINUTES TO
};




volatile uint8_t flags = 0;
volatile uint8_t ir_cmd;
uint8_t framesvmode = 0;
uint8_t br = 0;
uint8_t buf[] = { //initial state of RTC control registers
		1<<RTC_12_24 | 1<<RTC_MI, //12 hours format, interrupt on minute change
		0x00,	//initial control2
		1<<RTC_BLIE //interrupt on low battery
};

uint8_t hPosLast = 0xFF,
		mPosLast = 0xFF;
uint8_t uartRxBuf[3];
volatile uint8_t uartRxBuf_p = 0;
volatile uint8_t t0_extended_int;

void init(void){
	PORTD |= (1<<PD3); //pull up on int1
	//initialize interrupt 1 (interrupt from RTC)
	MCUCR |= (1<<ISC11); //on falling edge
	GICR |= (1<<INT1);
	mxcontrol_init(); //init display control
	ir_init(); //init IR-control
	i2c_init();
	uart_init();

	sei();
}

void mapToMX(const uint8_t* map, uint8_t pos){
	//seek
	while (pos){
		map+=(pgm_read_byte(map)>>4)<<1; //<<1 - это умножение на 2, т.к. одна линия кодируется 2 байтами. нет времени объяснять!!!
		pos -= 1;
	}
	//итак, map смотрит куда надо теперь
	pos = pgm_read_byte(map)>>4; //здесь pos хранит число линий, которое нужно отрисовать
	while (pos--){
		//X хранится в старшем октете 2 байта, Y - в младшем, длина линии - в владшем октете 1-го байта.
		//8кБ... мы экономили, как могли...
		mxcontrol_makehline(pgm_read_byte(map+1)>>4, pgm_read_byte(map+1) & 0x0F, pgm_read_byte(map) & 0x0F, 1);
		map+=2;
	}
}

void timeToMX (uint8_t h, uint8_t m){
	//normalize time. h & m in binary format
	uint8_t mMod, //
		mMapPos = 0; //position in minutes-map array
	for (mMod = m; mMod > 5; mMod-=5){ //mTmp в итоге остаток от деления на 5
		mMapPos++;
	}
	if (mMod>2) mMapPos++; //такое вот округление



	//для второй половины часа сдвигаем h вперед. ох и бля
	if(mMapPos >= 7) h++;

	//h-часов и 60 минут обрабатывает вот так:
		if(mMapPos == 12){
			mMapPos = 0;
			//h++;
		}
	//fix hour (когда инкрементом перешли 12-часовой рубеж.
	if (h >= 12) h-=12;

	if (h == hPosLast && mMapPos == mPosLast) return;
	hPosLast = h; mPosLast = mMapPos;

	mxcontrol_clear();
	mxcontrol_makehline(0, 0, 2, 1); //IT
	mxcontrol_makehline(3, 0, 2, 1); //IS

	mapToMX(hMap, h);
	mapToMX(mMap, mMapPos);

	mxcontrol_draw(framesvmode);
}

void ir_oncmdrecv(uint8_t cmd){
	ir_cmd = cmd;
	flags |= F_IR_CMD_RECEIVED;
}

void uart_onrecv(uint8_t d){
	if (flags & F_UART_CMD_RECEIVED) return; //если команда обрабатывается, то забиваем болт!
	if (d == 0xFF){
		//0xFF в любом месте сбрасывает все к ебеням!
		uartRxBuf_p = 0; //reset byte = 0xFF
	}else{
		uartRxBuf[uartRxBuf_p++] = d; //byte into buffer
	}
	if ((uartRxBuf[0] == 0x00 && uartRxBuf_p == 2) || (uartRxBuf[0] == 0x01 && uartRxBuf_p == 3)){
		//команда готова и можно парсить буфер в основном потоке
		flags |= F_UART_CMD_RECEIVED;
		uartRxBuf_p = 0; //поинтер буфера обнулим для след. команд.
	}
}

void test(void){


	mxcontrol_clear();
	mxcontrol_setprintcoords(2,2);
	mxcontrol_printchar('b');
	mxcontrol_printchar('r');
	mxcontrol_printchar('i');
	mxcontrol_draw(3);


	while(1);
}

int main(void){
	uint8_t t;

	init();
	_delay_ms(1000); //ждем какое-то время, чтоб дать RTC запуститься в стабильный режим
	mxcontrol_setbrightnessauto(1);
	//test();
	//init rtc here!!!
	i2c_reg_write(RTC_SLA, RTC_CONTROL_1, buf, 3);
	//test();
	//mxcontrol_setbrightness(br);

	flags |= F_RTC_MINCHANGED; //set flag to show current time on start (no interrupt)
	while(1){
		if (flags & F_IR_CMD_RECEIVED){
			flags &= ~F_IR_CMD_RECEIVED; //reset flag
			//uart_send(ir_cmd);
			switch(ir_cmd){
			case CMD_UP:
				//hour++
				i2c_reg_read(RTC_SLA, RTC_HOURS, &t, 1);
				t = bcd2bin(t & RTC_HOURS_MASK);
				if ((t++)==12) t=1;
				t = bin2bcd(t);
				i2c_reg_write(RTC_SLA, RTC_HOURS, &t, 1);
				flags |= F_RTC_MINCHANGED;
				break;
			case CMD_DOWN:
				//hour--
				i2c_reg_read(RTC_SLA, RTC_HOURS, &t, 1);
				t = bcd2bin(t & RTC_HOURS_MASK);
				if ((t--)==1) t=12;
				t = bin2bcd(t);
				i2c_reg_write(RTC_SLA, RTC_HOURS, &t, 1);
				flags |= F_RTC_MINCHANGED;
				break;
			case CMD_RIGHT:
				//min++
				t=0;
				i2c_reg_write(RTC_SLA, RTC_SECONDS, &t, 1);

				i2c_reg_read(RTC_SLA, RTC_MINUTES, &t, 1);
				t = bcd2bin(t & RTC_MINUTES_MASK);
				if ((t++)==59) t = 0;
				t = bin2bcd(t);
				i2c_reg_write(RTC_SLA, RTC_MINUTES, &t, 1);
				flags |= F_RTC_MINCHANGED;
				break;
			case CMD_LEFT:
				//min--
				t=0;
				i2c_reg_write(RTC_SLA, RTC_SECONDS, &t, 1);

				i2c_reg_read(RTC_SLA, RTC_MINUTES, &t, 1);
				t = bcd2bin(t & RTC_MINUTES_MASK);
				if ((t--)==0) t = 59;
				t = bin2bcd(t);
				i2c_reg_write(RTC_SLA, RTC_MINUTES, &t, 1);
				flags |= F_RTC_MINCHANGED;
				break;
			case CMD_PWR:
				if(framesvmode++ == 3) framesvmode = 0;
				break;
			case CMD_VOL_UP:
				if(br > 250) break;
				br+=5;
				mxcontrol_setbrightness(br);
				break;
			case CMD_VOL_DOWN:
				if(br < 5) break;
				br-=5;
				mxcontrol_setbrightness(br);
				break;
			case CMD_PLAY:
				mxcontrol_blink(1);
				break;
			case CMD_STOP:
				mxcontrol_blink(0);
				break;
			case CMD_MODE_CH:
				//mode changed

				//entermode();
				break;
			}
		}

		if (flags & F_RTC_MINCHANGED){
			flags &= ~F_RTC_MINCHANGED;
			//read current time
			i2c_reg_read(RTC_SLA, RTC_MINUTES, buf, 2);
			uart_send(buf[1]);//debug send hours
			uart_send(buf[0]);//debug send min
			timeToMX(bcd2bin(buf[1] & RTC_HOURS_MASK), bcd2bin(buf[0] & RTC_MINUTES_MASK));
		}
		if (flags & F_UART_CMD_RECEIVED){
			//парсим команду и делаем то, что от нас требуется
			switch (uartRxBuf[0]){
			case 0x00:
				//read register and show it's value
				i2c_reg_read(RTC_SLA, uartRxBuf[1], (uartRxBuf+2), 1); //прочитанное значение запишется в конец буфера [2]
				//uart_send(uartRxBuf[2]); //send value to terminal
				break;
			case 0x01:
				//write [2] to register with address [1]
				i2c_reg_write(RTC_SLA, uartRxBuf[1], (uartRxBuf+2), 1);
				//uart_send(uartRxBuf[2]); //для симетрии просто ответим тем, что пытались записать
				break;
			}
			uart_send(uartRxBuf[2]);
			//скидываем флаг в самом конце, иначе в буфер может прийти нежданчик
			flags &= ~F_UART_CMD_RECEIVED;
		}
	}
	return 0;
}

ISR(INT1_vect){
	//interrupt from RTC
	uint8_t status;
	//see what happened
	i2c_reg_read(RTC_SLA, RTC_CONTROL_2, &status, 1); //read c2
	if (status & (1<<RTC_MSF)){
		flags |= F_RTC_MINCHANGED;
		//reset interrupt flag in RTC-module
		status &= ~(1<<RTC_MSF);
		i2c_reg_write(RTC_SLA, RTC_CONTROL_2, &status, 1);
	}

	i2c_reg_read(RTC_SLA, RTC_CONTROL_3, &status, 1); //read c3
	if (status & (1<<RTC_BLF)){
		flags |= F_RTC_LOWBATT;
	}else{
		flags &= ~F_RTC_LOWBATT;
	}
}
