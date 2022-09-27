/*
 * bq25601.c
 *
 *  Created on: Sep 20, 2022
 *      Author: Kuprin_IV
 */
#include "bq25601.h"
#include "main.h"
#include "queue.h"
#include "keyboard.h"
#include "bt121.h"

// driver functions
static void BQ25601_init(void);
static void BQ25601_powerOff(void);
static void BQ25601_getState(BQ25601_Status* state);
static void BQ25601_getChargerFault(BQ25601_FaultType* fault);
static void BQ25601_setChargerEnabled(uint8_t is_enabled);

// inner functions
static void BQ25601_writeRegister(uint8_t addr, uint8_t value);
static uint8_t BQ25601_readRegister(uint8_t addr);

// init driver
BQ25601_Driver bq25601_driver = {
		BQ25601_init,
		BQ25601_powerOff,
		BQ25601_getState,
		BQ25601_getChargerFault,
		BQ25601_setChargerEnabled,
};

BQ25601_Driver* bq25601_drv = &bq25601_driver;

// init variables
extern I2C_HandleTypeDef hi2c2;
volatile uint8_t isChargerEnabled = 0;

/**
 * @brief Write BQ25601 register
 * @param: addr - register address
 * @param: value - register data
 */
static void BQ25601_writeRegister(uint8_t addr, uint8_t value)
{
	uint8_t data[2] = {0};
	data[0] = addr;
	data[1] = value;

	HAL_I2C_Master_Transmit(&hi2c2, BQ25601_SLAVE_ADDR, data, 2, 1000);
}

/**
 * @brief Read BQ25601 register
 * @param: addr - register address
 * @return: register data
 */
static uint8_t BQ25601_readRegister(uint8_t addr)
{
	uint8_t txData = 0;
	uint8_t rxData = 0;

	txData = addr;

	HAL_I2C_Master_Transmit(&hi2c2, BQ25601_SLAVE_ADDR, &txData, 1, 1000); // send register address
	HAL_I2C_Master_Receive(&hi2c2, BQ25601_SLAVE_ADDR, &rxData, 1, 1000); // receive register value
	return rxData;
}

/**
 * @brief Init BQ25601 charger parameters
 * @param: None
 * @return: None
 */
static void BQ25601_init(void)
{
	uint8_t temp_reg = 0;
	// set input current limit to 500 mA
	temp_reg = BQ25601_readRegister(REG00);
	if((temp_reg & 0x1F) != 0x05)
	{
		temp_reg &= 0xE0;
		temp_reg |= 0x05;
		BQ25601_writeRegister(REG00, temp_reg);
	}
	// set SYS_Min voltage to 3.2V
	temp_reg = BQ25601_readRegister(REG01);
	if((temp_reg & 0x0E) != 0x06)
	{
		temp_reg &= 0xF1;
		temp_reg |= 0x06;
		BQ25601_writeRegister(REG01, temp_reg);
	}
	// set charge current 360 mA
	temp_reg = BQ25601_readRegister(REG02);
	if((temp_reg & 0x3F) != 0x06)
	{
		temp_reg &= 0xC0;
		temp_reg |= 0x06;
		BQ25601_writeRegister(REG02, temp_reg);
	}

	// set pre-charge current 120 mA and termination current 60 mA
	temp_reg = BQ25601_readRegister(REG03);
	if((temp_reg & 0x10) != 0x10)
	{
		temp_reg = 0x10;
		BQ25601_writeRegister(REG03, temp_reg);
	}
	// set top-off timer to 15 min and rechargable threshold 200 mV
	temp_reg = BQ25601_readRegister(REG04);
	if((temp_reg & 0x07) != 0x03)
	{
		temp_reg &= 0xF8;
		temp_reg |= 0x03;
		BQ25601_writeRegister(REG04, temp_reg);
	}
	// disable watchdog timer
	temp_reg = BQ25601_readRegister(REG05);
	if((temp_reg & 0x30) != 0)
	{
		temp_reg &= 0xCF;
		BQ25601_writeRegister(REG05, temp_reg);
	}
	// set charger voltage to VREG = 4.208 V
	temp_reg = BQ25601_readRegister(REG07);
	if((temp_reg & 0x30) != 0x10)
	{
		temp_reg &= 0xCF;
		temp_reg |= 0x10;
		BQ25601_writeRegister(REG07, temp_reg);
	}
}

/**
 * @brief Power off function: disable BQ25601 charger BATFET
 * @param: None
 * @return: None
 */
static void BQ25601_powerOff(void)
{
	uint8_t reg07 = BQ25601_readRegister(REG07); // read REG07 data
	BQ25601_writeRegister(REG07, reg07 & 0xF7); // reset BATFET_DLY bit
	BQ25601_writeRegister(REG07, reg07 | 0x20); // set BATFET_DIS bit
}

/**
* @brief Get BQ25601 charger state
* @param: state - data structure with charger state
* @return: None
*/
static void BQ25601_getState(BQ25601_Status* state)
{
	uint8_t reg08 = BQ25601_readRegister(REG08); // read REG08 data
	// fill charger state data
	state->vbus_status = (reg08 & 0xE0)>>5;
	state->charger_status = (reg08 & 0x18)>>3;
	state->is_power_good = (reg08 & 0x04)>>2;
	state->is_vbat_low = (reg08 & 0x01);
}

/**
* @brief Get BQ25601 charger fault state
* @param: fault - data structure with charger fault state
* @return: None
*/
static void BQ25601_getChargerFault(BQ25601_FaultType* fault)
{
	// read REG09 data
	uint8_t reg09 = 0;
	reg09 = BQ25601_readRegister(REG09); // the first read reports the pre-existing fault register status
	reg09 = BQ25601_readRegister(REG09); // the second read reports the current fault register status.
	// fill charger fault state
	fault->is_watchdog_fault = (reg09 & 0x80)>>7;
	fault->is_boost_fault = (reg09 & 0x40)>>6;
	fault->charge_fault = (reg09 & 0x30)>>4;
	fault->is_bat_ovp = (reg09 & 0x08)>>3;
	fault->ntc_fault = (reg09 & 0x07);
}

/**
* @brief Set BQ25601 charger enabled state
* @param: is_enabled: 0 - charger disabled, 1 - charger enabled
* @return: None
*/
static void BQ25601_setChargerEnabled(uint8_t is_enabled)
{
	isChargerEnabled = is_enabled;
	if(is_enabled)
	{
		GHGEN_GPIO_Port->ODR &= ~GHGEN_Pin;
	}
	else
	{
		GHGEN_GPIO_Port->ODR |= GHGEN_Pin;
	}
}

// handle INT pulse event
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BQ25601_Status charger_status;
	BQ25601_FaultType charger_fault_state;
	uint8_t read_power_ic_status_cmd[9] = {0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

	if(GPIO_Pin == INT_Pin)
	{
		BQ25601_getState(&charger_status); // read charger status
		BQ25601_getChargerFault(&charger_fault_state); // read charger faults state
		if(charger_fault_state.charge_fault != 0 || charger_fault_state.is_bat_ovp != 0 || charger_fault_state.is_boost_fault != 0 ||
				charger_fault_state.is_watchdog_fault != 0 || charger_fault_state.ntc_fault != 0)
		{
			// fill command buffer to prevent incorrect commands set
			if(kbState->isDelayMeasureTestEnabled) read_power_ic_status_cmd[1] = 1;
			if(bt121_fw_update->startBTBootMode) read_power_ic_status_cmd[2] = 1;
			if(kbState->isDfuModeEnabled) read_power_ic_status_cmd[3] = 1;
			if(kbState->IsJoysticksCalibrationModeEnabled()) read_power_ic_status_cmd[4] = 1;
			if(isChargerEnabled) read_power_ic_status_cmd[5] = 1;
			// put get status command in queue
			commands_queue->Insert(read_power_ic_status_cmd);
		}
	}
}
