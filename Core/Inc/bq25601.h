/*
 * bq25601.h
 *
 *  Created on: Sep 20, 2022
 *      Author: Kuprin_IV
 */

#ifndef INC_BQ25601_H_
#define INC_BQ25601_H_

#include "stm32f4xx_hal.h"

#define BQ25601_SLAVE_ADDR	0xD6

// BQ25601 registers map
#define REG00				0x00
#define REG01				0x01
#define REG02				0x02
#define REG03				0x03
#define REG04				0x04
#define REG05				0x05
#define REG06				0x06
#define REG07				0x07
#define REG08				0x08
#define REG09				0x09
#define REG0A				0x0A
#define REG0B				0x0B

// BQ25601 status
#define NO_VBUS_INPUT		0x00
#define VBUS_USB_HOST		0x01
#define VBUS_ADAPTER		0x02
#define VBUS_USB_OTG		0x07
#define NOT_CHARGING		0x00
#define PRE_CHARGE			0x01
#define FAST_CHARGE			0x02
#define CHARGE_TERMINATION	0x03

// BQ25601 faults
#define WATCHDOG_FAULT		0x01
#define BOOST_FAULT			0x01
#define CHARGE_NORMAL		0x00
#define CHARGE_INPUT_FAULT	0x01
#define CHARGE_TERMAL_SHDN	0x02
#define BAT_OVP_FAULT		0x01
#define NTC_NORMAL			0x00
#define NTC_WARM			0x02
#define NTC_COOL			0x03
#define NTC_COLD			0x05
#define NTC_HOT				0x06

typedef struct
{
	uint8_t vbus_status;
	uint8_t charger_status;
	uint8_t is_power_good;
	uint8_t is_vbat_low;
}BQ25601_Status;

typedef struct
{
	uint8_t is_watchdog_fault;
	uint8_t is_boost_fault;
	uint8_t charge_fault;
	uint8_t is_bat_ovp;
	uint8_t ntc_fault;
}BQ25601_FaultType;

typedef struct
{
	void (*Init)(void);
	void (*PowerOff)(void);
	void (*GetChargerState)(BQ25601_Status* state);
	void (*GetChargerFault)(BQ25601_FaultType* fault);
	void (*SetChargerEnabled)(uint8_t is_enabled);
}BQ25601_Driver;

extern BQ25601_Driver* bq25601_drv;

#endif /* INC_BQ25601_H_ */
