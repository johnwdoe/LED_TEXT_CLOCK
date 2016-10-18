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

#define CMD_PWR (0x04)
#define CMD_TITLE (0x05)
#define CMD_ZOOM (0x06)
#define CMD_SRC (0x07)
#define CMD_SETUP (0x08)
#define CMD_UP (0x09)
#define CMD_SUB_T (0x0A)
#define CMD_REPEAT (0x0B)
#define CMD_LEFT (0x0C)
#define CMD_ENTER (0x0D)
#define CMD_RIGHT (0x0E)
#define CMD_A_B (0x0F)
#define CMD_PROG (0x10)
#define CMD_DOWN (0x11)
#define CMD_STOP (0x12)
#define CMD_PLAY (0x1C)
#define CMD_NUM_1 (0x19)
#define CMD_NUM_2 (0x18)
#define CMD_NUM_3 (0x14)
#define CMD_GOTO (0x13)
#define CMD_NUM_4 (0x59)
#define CMD_NUM_5 (0x15)
#define CMD_NUM_6 (0x1A)
#define CMD_MENU_PBC (0x16)
#define CMD_NUM_7 (0x5C)
#define CMD_NUM_8 (0x5D)
#define CMD_NUM_9 (0x17)
#define CMD_CSD_AMS (0x4D)
#define CMD_NUM_0 (0x1E)
#define CMD_SEEK_P (0x1D)
#define CMD_NUM_10_P (0x1B)
#define CMD_AUDIO (0x1F)
#define CMD_VOL_DOWN (0x54)
#define CMD_SEL (0x55)
#define CMD_VOL_UP (0x50)
#define CMD_DISP_ESC (0x51)
#define CMD_MUTE (0x44)
#define CMD_SEEK_M (0x49)
#define CMD_BAND_PN (0x48)
#define CMD_ANGLE (0x58)

void ir_init(void);
extern void ir_oncmdrecv(uint8_t cmd);

#endif /* IR_H_ */
