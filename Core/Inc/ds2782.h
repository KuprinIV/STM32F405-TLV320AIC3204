/*
 * ds2782.h
 *
 *  Created on: Sep 20, 2022
 *      Author: Kuprin_IV
 */

#ifndef INC_DS2782_H_
#define INC_DS2782_H_

#include "stm32f4xx_hal.h"

#define DS2782_SLAVE_ADDR		0x68

// Memory Map
#define STATUS_REG				0x01	// Status Register
#define RAAC_MSB_REG			0x02	// Remaining Active Absolute Capacity MSB
#define RAAC_LSB_REG			0x03	// Remaining Active Absolute Capacity LSB
#define RSAC_MSB_REG			0x04	// Remaining Standby Absolute Capacity MSB
#define RSAC_LSB_REG			0x05	// Remaining Standby Absolute Capacity LSB
#define RARC_REG				0x06	// Remaining Active Relative Capacity
#define RSRC_REG				0x07	// Remaining Standby Relative Capacity
#define IAVG_MSB_REG			0x08	// Average Current Register MSB
#define IAVG_LSB_REG			0x09	// Average Current Register LSB
#define TEMP_MSB_REG			0x0A	// Temperature Register MSB
#define TEMP_LSB_REG			0x0B	// Temperature Register LSB
#define VOLT_MSB_REG			0x0C	// Voltage Register MSB
#define VOLT_LSB_REG			0x0D	// Voltage Register LSB
#define CURRENT_MSB_REG			0x0E	// Current Register MSB
#define CURRENT_LSB_REG			0x0F	// Current Register LSB
#define ACR_MSB_REG				0x10	// Accumulated Current Register MSB
#define ACR_LSB_REG				0x11	// Accumulated Current Register LSB
#define ACRL_MSB_REG			0x12	// Low Accumulated Current Register MSB
#define ACRL_LSB_REG			0x13	// Low Accumulated Current Register LSB
#define AS_REG					0x14	// Age Scalar
#define SFR_REG					0x15	// Special Feature Register
#define FULL_CAPACITY_MSB_REG	0x16	// Full Capacity MSB
#define FULL_CAPACITY_LSB_REG	0x17	// Full Capacity LSB
#define AE_MSB_REG				0x18	// Active Empty MSB
#define AE_LSB_REG				0x19	// Active Empty LSB
#define SE_MSB_REG				0x1A	// Standby Empty MSB
#define SE_LSB_REG				0x1B	// Standby Empty LSB
#define EEPROM_REG				0x1F	// EEPROM Register
#define FUNCTION_CR_REG			0xFE	// Function Command Register

// Parameter EEPROM Memory Block 1
#define CONTROL_REG_MB			0x60	// Control Register
#define ACCUM_BIAS_MB			0x61	// Accumulation Bias
#define AGING_CAP_MSB_MB		0x62	// Aging Capacity MSB
#define AGING_CAP_LSB_MB		0x63	// Aging Capacity LSB
#define VCHG_MB					0x64	// Charge Voltage
#define IMIN_MB					0x65	// Minimum Charge Current
#define VAE_MB					0x66	// Active Empty Voltage
#define IAE_MB					0x67	// Active Empty Current
#define AE40_MB					0x68	// Active Empty 40
#define RSNSP_MB				0x69	// Sense Resistor Prime
#define FULL_40_MSB_MB			0x6A	// Full 40 MSB
#define FULL_40_LSB_MB			0x6B	// Full 40 LSB
#define FULL_3040_SLOPE_MB		0x6C	// Full 3040 Slope
#define FULL_2030_SLOPE_MB		0x6D	// Full 2030 Slope
#define FULL_1020_SLOPE_MB		0x6E	// Full 1020 Slope
#define FULL_0010_SLOPE_MB		0x6F	// Full 0010 Slope
#define AE_3040_SLOPE_MB		0x70	// AE 3040 Slope
#define AE_2030_SLOPE_MB		0x71	// AE 2030 Slope
#define AE_1020_SLOPE_MB		0x72	// AE 1020 Slope
#define AE_0010_SLOPE_MB		0x73	// AE 0010 Slope
#define SE_3040_SLOPE_MB		0x74	// SE 3040 Slope
#define SE_2030_SLOPE_MB		0x75	// SE 2030 Slope
#define SE_1020_SLOPE_MB		0x76	// SE 1020 Slope
#define SE_0010_SLOPE_MB		0x77	// SE 0010 Slope
#define RSGAIN_MSB_MB			0x78	// Sense Resistor Gain MSB
#define RSGAIN_LSB_MB			0x79	// Sense Resistor Gain LSB
#define RSTC_MB					0x7A	// Sense Resistor Temp. Coeff.
#define FRSGAIN_MSB_MB			0x7B	// Factory Gain MSB
#define FRSGAIN_LSB_MB			0x7C	// Factory Gain LSB
#define I2C_SLAVE_ADDR_MB		0x7E	// 2-Wire (I2C) Slave Address

// status bits values
#define CHARGE_TERMINATION_FLAG	0x80
#define ACTIVE_EMPTY_FLAG		0x40
#define STANDBY_EMPTY_FLAG		0x20
#define LEARN_FLAG				0x10
#define UNDER_VOLTAGE_FLAG		0x04
#define POWER_ON_RESET_FLAG		0x02

// function commands
/**
 * This command copies the shadow RAM to the target EEPROM block.
 */
#define COPY_DATA_CMD_BLK0		0x42
#define COPY_DATA_CMD_BLK1		0x44
/**
 * This command recalls the contents of the targeted EEPROM block to its shadow RAM.
 */
#define RECALL_DATA_CMD_BLK0	0xB2
#define RECALL_DATA_CMD_BLK1	0xB4
/**
 * This command locks (write-protects) the targeted EEPROM block.
 * The lock command is permanent; a locked block can never be written again.
 */
#define LOCK_CMD_BLK0			0x63
#define LOCK_CMD_BLK1			0x66

typedef struct
{
	void (*Init)(void);
	int16_t (*ReadBatteryVoltage)(void);
	int16_t (*ReadBatteryCurrent)(void);
	uint8_t (*ReadActiveRelativeCapacity)(void);
	uint8_t (*ReadStandbyRelativeCapacity)(void);
	uint8_t (*ReadStatus)(void);
	void (*ReadEepromBlock1)(uint8_t start_addr, uint8_t* data, uint8_t length);
	void (*WriteEepromBlock1)(uint8_t start_addr, uint8_t* data, uint8_t length);
	void (*LockEepromBlock)(uint8_t block_num);
	uint8_t (*IsEepromBlockLocked)(uint8_t block_num);
}DS2782_Driver;

typedef struct
{
	uint8_t Rsense;
	uint8_t Vcharge;
	uint8_t Imin;
	uint8_t VoltAE;
	uint8_t CurrentAE;
	uint16_t agingCapacity;
	uint16_t fullCapacity40;
	uint8_t activeEmpty40;
	uint8_t full3040_slope;
	uint8_t full2030_slope;
	uint8_t full1020_slope;
	uint8_t full0010_slope;
	uint8_t ae3040_slope;
	uint8_t ae2030_slope;
	uint8_t ae1020_slope;
	uint8_t ae0010_slope;
	uint8_t se3040_slope;
	uint8_t se2030_slope;
	uint8_t se1020_slope;
	uint8_t se0010_slope;
	uint16_t rsgain;
	uint8_t control_register;
}DS2782_InitParams;

extern DS2782_Driver* ds2782_drv;

#endif /* INC_DS2782_H_ */
