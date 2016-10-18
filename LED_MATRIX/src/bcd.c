/*
 * bcd.c
 *
 *  Created on: Jun 1, 2016
 *      Author: william
 */
#include "bcd.h"

uint8_t bin2bcd(uint8_t v){
	uint8_t i, res = 0x00;
	for (i=0; i<8; i++){
		//fix
		if ((res & 0xF0) > 0x40) res += 0x30;
		if ((res & 0x0F) > 0x04) res += 0x03;
		//shift
		res <<= 1;
		if (v & 0x80) res |= 0x01;
		v<<=1;


	}
	return res;
}

uint8_t bcd2bin(uint8_t v){
	uint8_t i, res = 0x00;
	for (i=0; i<8; i++){
		res >>=1; //shift result
		if (v & 0x01) res |= 0x80; //bit to result
		v >>= 1; //shift argument

		if ((v & 0xF0) > 0x40) v -= 0x30;
		if ((v & 0x0F) > 0x04) v -= 0x03;
	}
	return res;
}

