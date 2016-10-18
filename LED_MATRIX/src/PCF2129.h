/*
 * PCF2129.h
 *
 *  Created on: Jun 1, 2016
 *      Author: william
 */

#ifndef PCF2129_H_
#define PCF2129_H_

#define RTC_SLA (0b01010001)

#define RTC_CONTROL_1 (0x00)
#define RTC_12_24 (2)
#define RTC_MI (1)

#define RTC_CONTROL_2 (0x01)
#define RTC_MSF (7)

#define RTC_CONTROL_3 (0x02)
#define RTC_BLF (2)
#define RTC_BLIE (0)

#define RTC_SECONDS (0x03)
#define RTC_SECONDS_MASK (0x7F)
#define RTC_MINUTES (0x04)
#define RTC_MINUTES_MASK (0x7F)
#define RTC_HOURS (0x05)
#define RTC_PM_MASK (1<<5)
#define RTC_HOURS_MASK (0x1F)

#endif /* PCF2129_H_ */
