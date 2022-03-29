#ifndef __BT121_H
#define __BT121_H

#include <stdint.h>

#define BT_CONNECTION_STATUS_HEADER  	0x81
#define BT_OUTPUT_REPORT_DATA_HEADER 	0x82
#define BT_INPUT_REPORT_DATA_HEADER  	0x01
#define BT_COMMAND_DATA_HEADER 		 	0x02
#define BT_OPEN_CONNECTION_CMD			0x01
#define BT_CLOSE_CONNECTION_CMD			0x02
#define BT_DELETE_BONDING_CMD			0x03

#define BT_START_FLASH_ADDR 			0x08000000

#define ACK 0x79
#define NACK 0x1F

#define BOOT_CMD_GET_CMD        0x00
#define BOOT_CMD_GET_VS         0x01
#define BOOT_CMD_GET_ID         0x02
#define BOOT_CMD_RD_MEM         0x11
#define BOOT_CMD_GO             0x21
#define BOOT_CMD_WR_MEM         0x31
#define BOOT_CMD_ERASE          0x43
#define BOOT_CMD_ERASE_EX       0x44
#define BOOT_CMD_WR_PROT        0x63
#define BOOT_CMD_WR_UNPROT      0x73
#define BOOT_CMD_RD_PROT        0x82
#define BOOT_CMD_RD_UNPROT      0x92

typedef enum {
  FLASH_ERASE_UNDEF,
  FLASH_ERASE_OLD,
  FLASH_ERASE_EXT
} FLASH_ERASE_TYPE;

typedef enum
{
	BT_NOT_CONNECTED = 0,
	BT_IS_CONNECTING,
	BT_IS_CONNECTED,
}BT121_Connection_Status;

typedef enum
{
	BT_WRITE_BOOT_CMD_ERROR = 0x04,
	BT_WRITE_ADDRESS_ERROR = 0x05,
	BT_WRITE_BOOT_DATA_ERROR = 0x06,
	BT_VERIFY_BOOT_CMD_ERROR = 0x07,
	BT_VERIFY_ADDRESS_ERROR = 0x08,
	BT_VERIFY_BOOT_READ_DATA_ERROR = 0x09,
	BT_FLASH_DATA_UNMATCH = 0x0A,

}BT121_FlashErrors;

typedef struct
{
	void (*Init)(void);
	void (*Reset)(void);
	void (*SetEnabled)(uint8_t is_enabled);
	uint8_t (*BootModeCtrl)(uint8_t mode);
	void (*SendInputReport)(uint8_t report_id, uint8_t* report_data, uint8_t report_length);
	uint8_t (*IsHID_EndpointConnected)(void);
	void (*DeleteBonding)(void);
	uint8_t (*IsBootModeEnabled)(void);
	uint8_t (*BT_FlashErase)(void);
	uint8_t (*BT_FlashWrite)(uint32_t flash_addr, uint8_t* data, uint16_t size);
	uint8_t (*BT_FlashVerify)(uint32_t flash_addr, uint8_t* check_data, uint16_t check_data_size);
}BT121_Drv;

extern BT121_Drv *bt121_drv;

#endif
