#ifndef __EEPROM_EMULATION_H
#define __EEPROM_EMULATION_H

#include "stm32f4xx_hal.h"

#define PAGE_DATA_OFFSET                                                4
#define PAGE_DATA_SIZE                                                  4

#define PAGE_0_SECTOR                                                  	FLASH_SECTOR_10
#define PAGE_1_SECTOR                                                  	FLASH_SECTOR_11
#define PAGE_SIZE                                                       131072 // sector size 128 kB
#define PAGES_NUM 														2

// define variables number
#define VAR_NUM															12
// define variables for left joystick calibration data
#define JOYSTICK_LEFT_H_AXIS_MIN_KEY									0x8000
#define JOYSTICK_LEFT_H_AXIS_MAX_KEY									0x8001
#define JOYSTICK_LEFT_H_AXIS_ZERO_KEY									0x8002
#define JOYSTICK_LEFT_V_AXIS_MIN_KEY									0x8003
#define JOYSTICK_LEFT_V_AXIS_MAX_KEY									0x8004
#define JOYSTICK_LEFT_V_AXIS_ZERO_KEY									0x8005
// define variables for right joystick calibration data
#define JOYSTICK_RIGHT_H_AXIS_MIN_KEY									0x8006
#define JOYSTICK_RIGHT_H_AXIS_MAX_KEY									0x8007
#define JOYSTICK_RIGHT_H_AXIS_ZERO_KEY									0x8008
#define JOYSTICK_RIGHT_V_AXIS_MIN_KEY									0x8009
#define JOYSTICK_RIGHT_V_AXIS_MAX_KEY									0x800A
#define JOYSTICK_RIGHT_V_AXIS_ZERO_KEY									0x800B

/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     	((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     	((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     	((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     	((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     	((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     	((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     	((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     	((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     	((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     	((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    	((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    	((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

#define CONSTANTS_SECTOR  			FLASH_SECTOR_9
#define CONSTANTS_SECTOR_ADDRESS  	ADDR_FLASH_SECTOR_9
#define DFU_SIGNATURE_ADDRESS  		CONSTANTS_SECTOR_ADDRESS 		// 0x080A0000
#define SERIAL_NUMBER_ADDRESS  		(CONSTANTS_SECTOR_ADDRESS+4) 	// 0x080A0004
#define FW_VERSION_ADDRESS  		(CONSTANTS_SECTOR_ADDRESS+12)	// 0x080A000C

typedef enum {
  PAGE_CLEARED = 0xFFFFFFFF,
  PAGE_ACTIVE = 0x00000000,
  PAGE_RECEIVING_DATA = 0x55555555,
} PageState;

typedef enum {
  PAGE_0 = 0,
  PAGE_1 = 1,
} PageIdx;

typedef enum {
  EEPROM_OK = 0,
  EEPROM_ERROR = 1,
} EepromResult;

typedef struct
{
	EepromResult (*Init)(void);
	EepromResult (*Read)(uint16_t varId, uint16_t *varValue);
	EepromResult (*Write)(uint16_t varId, uint16_t varValue);
	EepromResult (*ResetDfuSignature)(void);
	EepromResult (*GetSerialNumber)(uint8_t* sn_buf, uint8_t length);
	EepromResult (*GetFwVersion)(uint8_t* fw_ver_buf, uint8_t length);
}EepromDrv;

extern EepromDrv* eeprom_drv;

#endif
