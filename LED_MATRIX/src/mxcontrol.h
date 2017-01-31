/*
 * mxcontrol.h
 *
 *  Created on: May 4, 2016
 *      Author: william
 */

#ifndef MXCONTROL_H_
#define MXCONTROL_H_
#include <stdint.h>

#define DISPLAY_PORT (PORTB)
#define DISPLAY_DDR (DDRB)
#define MOSI_P	(PB3)
#define SCK_P	(PB5)
#define STB_P	(PB0)
#define OE_P	(PB2)
//#define RST_P	(PB5)
#define PWM_MIN (0)

#define T0_OVF_F_2	(1)
#define T0_OVF_F_4	(1<<1)
#define T0_OVF_F_8	(1<<2)
#define T0_OVF_F_16	(1<<3)
#define T0_OVF_F_32	(1<<4)
#define T0_OVF_F_64	(1<<5)
#define T0_OVF_F_128	(1<<6)
#define T0_OVF_F_256	(1<<7)

#define MIN_AUTOBRIGHTNESS_TRESHOLD	(15)
#define MAX_AUTOBRIGHTNESS_TRESHOLD	(100)

/*
 * инициализация дисплея (включая IO)
 */
void mxcontrol_init(void);

/*
 * установка яркости (скважности ШИМ)
 * 0 - минимум, 255 - максимум
 */
void mxcontrol_setbrightness (uint8_t b);

/*
 * автоматическая яркость
 * 0 - выключена
 * !0 - включена
 */
void mxcontrol_setbrightnessauto(uint8_t b);

/*
 * рисует точку цвета color с координатами x, y в буфере
 * x - слева направо
 * y - сверху вниз
 * color:	0 - выключен
 * 			!0 - включен
 */
void mxcontrol_makepoint(uint8_t x, uint8_t y, uint8_t color);

/*
 * рисует в буфере горизонтальную линию цвета color из точки (x, y) длиной lenght
 * линия рисуется слева-направо
 */
void mxcontrol_makehline(uint8_t x, uint8_t y, uint8_t lenght, uint8_t color);

/*
 * очищает буфер
 */
void mxcontrol_clear(void);

/*
 * копирует буфер в видеопамять (отображает)
 * mode задает способ копирования (эдакий спецэффект)
 */
void mxcontrol_draw(uint8_t mode);

/*
 * управение режимом мигания (blink)
 * 0 - включен
 * !0 - выключен
 */
void mxcontrol_blink(uint8_t b);

/*
 * устанавливает стартовые координаты дляпечати символа (левый верхний угол)
 */
void mxcontrol_setprintcoords(uint8_t x, uint8_t y);

/*
 * печать символа в буфер
 * c - индекс в алфавите (не код символа!!!)
 */
void mxcontrol_print(uint8_t c);

/*
 * то же самое,но с ASCII-кодом
 */
void mxcontrol_printchar (char c);


#endif /* MXCONTROL_H_ */
