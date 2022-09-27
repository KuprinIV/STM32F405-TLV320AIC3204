#include "ds2782.h"

// driver functions
static void DS2782_init(void);
static int16_t DS2782_readBatteryVoltage(void);
static int16_t DS2782_readBatteryCurrent(void);
static uint8_t DS2782_readActiveRelativeCapacity(void);
static uint8_t DS2782_readStandbyRelativeCapacity(void);
static uint8_t DS2782_readStatus(void);
static void DS2782_readEepromBlock1(uint8_t start_addr, uint8_t* data, uint8_t length);
static void DS2782_writeEepromBlock1(uint8_t start_addr, uint8_t* data, uint8_t length);
static void DS2782_lockEepromBlock(uint8_t block_num);
static uint8_t DS2782_isEepromBlockLocked(uint8_t block_num);

// inner functions
static void DS2782_writeRegister(uint8_t addr, uint8_t value);
static uint8_t DS2782_readRegister(uint8_t addr);
static void DS2782_copyDataEepromBlock(uint8_t block_num);
static void DS2782_recallDataEepromBlock(uint8_t block_num);

// init driver
DS2782_Driver ds2782_driver = {
		DS2782_init,
		DS2782_readBatteryVoltage,
		DS2782_readBatteryCurrent,
		DS2782_readActiveRelativeCapacity,
		DS2782_readStandbyRelativeCapacity,
		DS2782_readStatus,
		DS2782_readEepromBlock1,
		DS2782_writeEepromBlock1,
		DS2782_lockEepromBlock,
		DS2782_isEepromBlockLocked,
};
DS2782_Driver* ds2782_drv = &ds2782_driver;

extern I2C_HandleTypeDef hi2c2;

/**
 * @brief Write DS2782 register
 * @param: addr - register address
 * @param: value - register data
 */
static void DS2782_writeRegister(uint8_t addr, uint8_t value)
{
	uint8_t data[2] = {0};
	data[0] = addr;
	data[1] = value;

	HAL_I2C_Master_Transmit(&hi2c2, DS2782_SLAVE_ADDR, data, 2, 1000);
}

/**
 * @brief Read DS2782 register
 * @param: addr - register address
 * @return: register data
 */
static uint8_t DS2782_readRegister(uint8_t addr)
{
	uint8_t txData = 0;
	uint8_t rxData = 0;

	txData = addr;

	HAL_I2C_Master_Transmit(&hi2c2, DS2782_SLAVE_ADDR, &txData, 1, 1000); // send register address
	HAL_I2C_Master_Receive(&hi2c2, DS2782_SLAVE_ADDR, &rxData, 1, 1000); // receive register value
	return rxData;
}

/**
 * @brief Init battery parameters and save them in DS2782 EEPPROM
 * @param: None
 * @return: battery_voltage_value*100
 */
static void DS2782_init(void)
{
	DS2782_InitParams ds2782_init;
	uint8_t temp8 = 0;
	uint16_t temp16 = 0;

	if(DS2782_isEepromBlockLocked(1)) return;

	// store battery parameters in data structure
	ds2782_init.Rsense = 21; 				// set Rsns = 47,6 mOhm in mhOms
	ds2782_init.rsgain = 1011;				// set Rsgain = 1011/2048 = 0,987 for adjust Rsns value close to 47 mOhm
	ds2782_init.Vcharge = 215; 				// set Vchg = 4,2 V in 19,52 mV steps. Uses for detection full-charge state
	ds2782_init.Imin = 75; 					// set Imin = 80 mA with Rsns = 47 mOhm. Uses for detection full-charge state
	ds2782_init.VoltAE = 169; 				// set Active Empty voltage to 3,3 V in 19.52 mV steps. Uses for detecting Active Empty state
	ds2782_init.CurrentAE = 12; 			// set Active Empty current 50 mA. Uses for detecting Active Empty state
	ds2782_init.agingCapacity = 5414; 		// set battery capacity 33,84 mVh in 6,25 uVh steps (equals 720 mA with Rsns = 47 mOhm)
	ds2782_init.control_register = 0x40;	// enable transition in Sleep mode, if VIN < Vsleep AND SDA, SCL stable at either logic level for Tsleep
	// battery model characteristics got close to characteristics from DS2782 datasheet example
	ds2782_init.fullCapacity40 = 5414; 		// set full battery capacity at 40°C 33,84 mVh in 6,25 uVh steps (equals 720 mA with Rsns = 47 mOhm)
	ds2782_init.activeEmpty40 = 41;			// 0,04: fraction of full battery capacity at 40°C in 1/1024 units
	ds2782_init.full3040_slope = 15;		// 915 ppm/°C: line slope between 30 and 40 °C in 61 ppm/°C units
	ds2782_init.full2030_slope = 28;		// 1708 ppm/°C: line slope between 20 and 30 °C in 61 ppm/°C units
	ds2782_init.full1020_slope = 38;		// 2318 ppm/°C: line slope between 10 and 20 °C in 61 ppm/°C units
	ds2782_init.full0010_slope = 39;		// 2379 ppm/°C: line slope between 0 and 10 °C in 61 ppm/°C units
	ds2782_init.ae3040_slope = 6;			// 366 ppm/°C: line slope between 30 and 40 °C in 61 ppm/°C units
	ds2782_init.ae2030_slope = 16; 			// 976 ppm/°C: line slope between 20 and 30 °C in 61 ppm/°C units
	ds2782_init.ae1020_slope = 30;			// 1830 ppm/°C: line slope between 10 and 20 °C in 61 ppm/°C units
	ds2782_init.ae0010_slope = 18;			// 1098 ppm/°C: line slope between 0 and 10 °C in 61 ppm/°C units
	ds2782_init.se3040_slope = 2;			// 122 ppm/°C: line slope between 30 and 40 °C in 61 ppm/°C units
	ds2782_init.se2030_slope = 5; 			// 305 ppm/°C: line slope between 20 and 30 °C in 61 ppm/°C units
	ds2782_init.se1020_slope = 5;			// 305 ppm/°C: line slope between 10 and 20 °C in 61 ppm/°C units
	ds2782_init.se0010_slope = 10;			// 610 ppm/°C: line slope between 0 and 10 °C in 61 ppm/°C units

	// check battery parameters in DS2782 EEPROM
	DS2782_recallDataEepromBlock(1); // recall data from EEPROM block 1 into shadow RAM
	temp8 = DS2782_readRegister(RSNSP_MB);
	if(temp8 != ds2782_init.Rsense)
	{
		DS2782_writeRegister(RSNSP_MB, ds2782_init.Rsense);
	}

	temp16 = (uint16_t)((DS2782_readRegister(RSGAIN_MSB_MB)<<8) | DS2782_readRegister(RSGAIN_LSB_MB));
	if(temp16 != ds2782_init.rsgain)
	{
		DS2782_writeRegister(RSGAIN_MSB_MB, (uint8_t)((ds2782_init.rsgain & 0xFF00)>>8));
		DS2782_writeRegister(RSGAIN_LSB_MB, (uint8_t)(ds2782_init.rsgain & 0xFF));
	}

	temp8 = DS2782_readRegister(VCHG_MB);
	if(temp8 != ds2782_init.Vcharge)
	{
		DS2782_writeRegister(VCHG_MB, ds2782_init.Vcharge);
	}

	temp8 = DS2782_readRegister(IMIN_MB);
	if(temp8 != ds2782_init.Imin)
	{
		DS2782_writeRegister(IMIN_MB, ds2782_init.Imin);
	}

	temp8 = DS2782_readRegister(VAE_MB);
	if(temp8 != ds2782_init.VoltAE)
	{
		DS2782_writeRegister(VAE_MB, ds2782_init.VoltAE);
	}

	temp8 = DS2782_readRegister(IAE_MB);
	if(temp8 != ds2782_init.CurrentAE)
	{
		DS2782_writeRegister(IAE_MB, ds2782_init.CurrentAE);
	}

	temp16 = (uint16_t)((DS2782_readRegister(AGING_CAP_MSB_MB)<<8) | DS2782_readRegister(AGING_CAP_LSB_MB));
	if(temp16 != ds2782_init.agingCapacity)
	{
		DS2782_writeRegister(AGING_CAP_MSB_MB, (uint8_t)((ds2782_init.agingCapacity & 0xFF00)>>8));
		DS2782_writeRegister(AGING_CAP_LSB_MB, (uint8_t)(ds2782_init.agingCapacity & 0xFF));
	}

	temp8 = DS2782_readRegister(CONTROL_REG_MB);
	if(temp8 != ds2782_init.control_register)
	{
		DS2782_writeRegister(CONTROL_REG_MB, ds2782_init.control_register);
	}

	temp16 = (uint16_t)((DS2782_readRegister(FULL_40_MSB_MB)<<8) | DS2782_readRegister(FULL_40_LSB_MB));
	if(temp16 != ds2782_init.fullCapacity40)
	{
		DS2782_writeRegister(FULL_40_MSB_MB, (uint8_t)((ds2782_init.fullCapacity40 & 0xFF00)>>8));
		DS2782_writeRegister(FULL_40_LSB_MB, (uint8_t)(ds2782_init.fullCapacity40 & 0xFF));
	}

	temp8 = DS2782_readRegister(AE40_MB);
	if(temp8 != ds2782_init.activeEmpty40)
	{
		DS2782_writeRegister(AE40_MB, ds2782_init.activeEmpty40);
	}

	temp8 = DS2782_readRegister(FULL_3040_SLOPE_MB);
	if(temp8 != ds2782_init.full3040_slope)
	{
		DS2782_writeRegister(FULL_3040_SLOPE_MB, ds2782_init.full3040_slope);
	}

	temp8 = DS2782_readRegister(FULL_2030_SLOPE_MB);
	if(temp8 != ds2782_init.full2030_slope)
	{
		DS2782_writeRegister(FULL_2030_SLOPE_MB, ds2782_init.full2030_slope);
	}

	temp8 = DS2782_readRegister(FULL_1020_SLOPE_MB);
	if(temp8 != ds2782_init.full1020_slope)
	{
		DS2782_writeRegister(FULL_1020_SLOPE_MB, ds2782_init.full1020_slope);
	}

	temp8 = DS2782_readRegister(FULL_0010_SLOPE_MB);
	if(temp8 != ds2782_init.full0010_slope)
	{
		DS2782_writeRegister(FULL_0010_SLOPE_MB, ds2782_init.full0010_slope);
	}

	temp8 = DS2782_readRegister(AE_3040_SLOPE_MB);
	if(temp8 != ds2782_init.ae3040_slope)
	{
		DS2782_writeRegister(AE_3040_SLOPE_MB, ds2782_init.ae3040_slope);
	}

	temp8 = DS2782_readRegister(AE_2030_SLOPE_MB);
	if(temp8 != ds2782_init.ae2030_slope)
	{
		DS2782_writeRegister(AE_2030_SLOPE_MB, ds2782_init.ae2030_slope);
	}

	temp8 = DS2782_readRegister(AE_1020_SLOPE_MB);
	if(temp8 != ds2782_init.ae1020_slope)
	{
		DS2782_writeRegister(AE_1020_SLOPE_MB, ds2782_init.ae1020_slope);
	}

	temp8 = DS2782_readRegister(AE_0010_SLOPE_MB);
	if(temp8 != ds2782_init.ae0010_slope)
	{
		DS2782_writeRegister(AE_0010_SLOPE_MB, ds2782_init.ae0010_slope);
	}

	temp8 = DS2782_readRegister(SE_3040_SLOPE_MB);
	if(temp8 != ds2782_init.se3040_slope)
	{
		DS2782_writeRegister(SE_3040_SLOPE_MB, ds2782_init.se3040_slope);
	}

	temp8 = DS2782_readRegister(SE_2030_SLOPE_MB);
	if(temp8 != ds2782_init.se2030_slope)
	{
		DS2782_writeRegister(SE_2030_SLOPE_MB, ds2782_init.se2030_slope);
	}

	temp8 = DS2782_readRegister(SE_1020_SLOPE_MB);
	if(temp8 != ds2782_init.se1020_slope)
	{
		DS2782_writeRegister(SE_1020_SLOPE_MB, ds2782_init.se1020_slope);
	}

	temp8 = DS2782_readRegister(SE_0010_SLOPE_MB);
	if(temp8 != ds2782_init.se0010_slope)
	{
		DS2782_writeRegister(SE_0010_SLOPE_MB, ds2782_init.se0010_slope);
	}

	DS2782_copyDataEepromBlock(1); // copy data from shadow RAM into EEPROM block 1
}

/**
 * @brief Read battery voltage from DS2782
 * @param: None
 * @return: battery_voltage_value*100
 */
static int16_t DS2782_readBatteryVoltage(void)
{
	int16_t volt_data_code = 0;
	uint8_t volt_msb_reg = DS2782_readRegister(VOLT_MSB_REG);
	uint8_t volt_lsb_reg = DS2782_readRegister(VOLT_LSB_REG);

	volt_data_code = (int16_t)((volt_msb_reg<<8)|volt_lsb_reg);
	volt_data_code = (volt_data_code>>5)*488/1000; // convert to volts*100 value
	return volt_data_code;
}

/**
 * @brief Read battery current from DS2782
 * @param: None
 * @return: battery_current_value in mA. If current < 0 - battery discharge, else - battery charge
 */
static int16_t DS2782_readBatteryCurrent(void)
{
	int16_t current_data_code = 0;
	int32_t temp = 0;
	uint8_t current_msb_reg = DS2782_readRegister(CURRENT_MSB_REG);
	uint8_t current_lsb_reg = DS2782_readRegister(CURRENT_LSB_REG);

	current_data_code = (int16_t)((current_msb_reg<<8)|(current_lsb_reg));

	// convert value to mA. 51200 - voltage range (51,2 mV) on Rs * 1000, 47 - Rs value in mOhms
	temp = (int32_t)(current_data_code*51200/47)>>15;
	return temp;
}

/**
 * @brief Read remaining active relative capacity from DS2782
 * @param: None
 * @return: Remaining active relative capacity value
 */
static uint8_t DS2782_readActiveRelativeCapacity(void)
{
	return DS2782_readRegister(RARC_REG);
}

/**
 * @brief Read remaining standby relative capacity from DS2782
 * @param: None
 * @return: Remaining standby relative capacity value
 */
static uint8_t DS2782_readStandbyRelativeCapacity(void)
{
	return DS2782_readRegister(RSRC_REG);
}

/**
 * @brief Read status register DS2782
 * @param: None
 * @return: Status register value
 */
static uint8_t DS2782_readStatus(void)
{
	return DS2782_readRegister(STATUS_REG);
}

/**
 * @brief Read data from EEPROM block 1 of DS2782
 * @param: start_addr - EEPROM start address
 * @param: data - output EEPROM data
 * @param: length - data array length
 * @return: None
 */
static void DS2782_readEepromBlock1(uint8_t start_addr, uint8_t* data, uint8_t length)
{
	uint8_t end_addr = 0x7F;
	// check start read address
	if(start_addr < 0x60 && start_addr > 0x7F) return;
	DS2782_recallDataEepromBlock(1); // recall data from EEPROM block 1 into shadow RAM
	if(start_addr+length < 0x7F)
	{
		end_addr = start_addr+length;
	}
	// fill out data array
	for(uint8_t i = 0; i < end_addr-start_addr; i++)
	{
		data[i] = DS2782_readRegister(start_addr+i);
	}
}

/**
 * @brief Write data into EEPROM block 1 of DS2782
 * @param: start_addr - EEPROM start address
 * @param: data - input EEPROM data
 * @param: length - data array length
 * @return: None
 */
static void DS2782_writeEepromBlock1(uint8_t start_addr, uint8_t* data, uint8_t length)
{
	uint8_t end_addr = 0x7F;
	// check start read address
	if(start_addr < 0x60 && start_addr > 0x7F) return;
	DS2782_recallDataEepromBlock(1); // recall data from EEPROM block 1 into shadow RAM
	if(start_addr+length < 0x7F)
	{
		end_addr = start_addr+length;
	}
	// write data array into EEPROM
	for(uint8_t i = 0; i < end_addr-start_addr; i++)
	{
		DS2782_writeRegister(start_addr+i, data[i]);
	}
	DS2782_copyDataEepromBlock(1); // copy data from shadow RAM into EEPROM block 1
}

/**
 * @brief Execute lock EEPROM memory block in DS2782. Lock command is permanent!
 * @param: block_num - EEPROM block number
 * @return: None
 */
static void DS2782_lockEepromBlock(uint8_t block_num)
{
	uint8_t eeprom_reg = DS2782_readRegister(EEPROM_REG);
	// check LOCK bit in EEPROM register
	if((eeprom_reg & 0x40) == 0)
	{
		// if LOCK bit is 0, set it to 1
		DS2782_writeRegister(EEPROM_REG, eeprom_reg|0x40);
	}
	// send lock function command
	if(block_num == 0)
	{
		DS2782_writeRegister(FUNCTION_CR_REG, LOCK_CMD_BLK0);
	}
	else if(block_num == 1)
	{
		DS2782_writeRegister(FUNCTION_CR_REG, LOCK_CMD_BLK1);
	}
	else // EEPROM block number is incorrect
	{
		return;
	}

	HAL_Delay(5); // wait command executing
}

/**
 * @brief Check is EEPROM memory block in DS2782 locked
 * @param: block_num - EEPROM block number
 * @return: 0 - EEPROM block isn't locked, 1 - EEPROM block is locked
 */
static uint8_t DS2782_isEepromBlockLocked(uint8_t block_num)
{
	uint8_t eeprom_reg = DS2782_readRegister(EEPROM_REG);
	uint8_t result = 0;

	if(block_num == 0)
	{
		if(eeprom_reg & 0x01) result = 1;
	}
	else if(block_num == 1)
	{
		if(eeprom_reg & 0x02) result = 1;
	}
	return result;
}

/**
 * @brief Execute copy shadow RAM data into EEPROM memory block in DS2782.
 * @param: block_num - EEPROM block number
 * @return: None
 */
static void DS2782_copyDataEepromBlock(uint8_t block_num)
{
	// send copy function command
	if(block_num == 0)
	{
		DS2782_writeRegister(FUNCTION_CR_REG, COPY_DATA_CMD_BLK0);
	}
	else if(block_num == 1)
	{
		DS2782_writeRegister(FUNCTION_CR_REG, COPY_DATA_CMD_BLK1);
	}
}

/**
 * @brief Execute recall EEPROM memory block data into shadow RAM in DS2782.
 * @param: block_num - EEPROM block number
 * @return: None
 */
static void DS2782_recallDataEepromBlock(uint8_t block_num)
{
	// send recall function command
	if(block_num == 0)
	{
		DS2782_writeRegister(FUNCTION_CR_REG, RECALL_DATA_CMD_BLK0);
	}
	else if(block_num == 1)
	{
		DS2782_writeRegister(FUNCTION_CR_REG, RECALL_DATA_CMD_BLK1);
	}
}
