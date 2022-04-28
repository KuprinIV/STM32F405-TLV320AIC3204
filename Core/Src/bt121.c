#include "bt121.h"
#include "keyboard.h"
#include "main.h"
#include <string.h>

// driver functions
static void BT121_Init(void);
static void BT121_Reset(void);
static void BT121_SetEnabled(uint8_t is_enabled);
static uint8_t BT121_BootModeCtrl(uint8_t mode);
static void BT121_SendInputReport(uint8_t report_id, uint8_t* report_data, uint8_t report_length);
static uint8_t BT121_IsHID_EndpointConnected(void);
static void BT121_DeleteBonding(void);
static uint8_t BT121_IsBootModeEnabled(void);
static uint8_t BT121_FlashErase(void);
static uint8_t BT121_FlashWrite(uint32_t flash_addr, uint8_t* data, uint16_t size);
static uint8_t BT121_FlashVerify(uint32_t flash_addr, uint8_t* check_data, uint16_t check_data_size);
// inner functions
static void BT121_ProcessOutputReport(uint8_t report_id, uint8_t* report_data, uint8_t report_length);
static uint8_t BT121_WriteBootCmd(uint8_t cmd);
static uint8_t BT121_WriteBootData(uint8_t *pfirst_byte, uint8_t *pbuff, uint16_t size);
static uint8_t BT121_GetCmdsList(FLASH_ERASE_TYPE *pflash_erase_type);
static uint8_t BT121_ReconfigureUART(UART_HandleTypeDef* huart, uint8_t boot_mode);

/** Buffer for storing data from the serial port. */
static uint8_t data_buffer[11];

/** HID connection params */
uint8_t is_hid_connected = 0;

/** BT inner parameters */
volatile uint8_t is_boot_enabled = 0; // BT boot mode state
volatile uint8_t is_transfer_ended = 0; // transfer end event flag

extern UART_HandleTypeDef huart3;

BT121_Drv bt121_drv_ctrl =
{
		BT121_Init,
		BT121_Reset,
		BT121_SetEnabled,
		BT121_BootModeCtrl,
		BT121_SendInputReport,
		BT121_IsHID_EndpointConnected,
		BT121_DeleteBonding,
		BT121_IsBootModeEnabled,
		BT121_FlashErase,
		BT121_FlashWrite,
		BT121_FlashVerify,
};
// init external driver structure
BT121_Drv *bt121_drv = &bt121_drv_ctrl;

/**
 * @brief Init BT121 module and BGAPI
 */
static void BT121_Init(void)
{
  // reset BT module
  BT121_Reset();
  // start data reception
  HAL_UARTEx_ReceiveToIdle_IT(&huart3, data_buffer, sizeof(data_buffer));
}

/**
 * @brief Reset BT121 module
 */
static void BT121_Reset(void)
{
  // reset BT module
  BT_RST_GPIO_Port->BSRR = BT_RST_Pin<<16;
  HAL_Delay(5);
  BT_RST_GPIO_Port->BSRR = BT_RST_Pin;
}

/**
 * @brief BT121 enable state control
 * @params
 * mode: 0 - module disabled, 1 - module enabled
 */
static void BT121_SetEnabled(uint8_t is_enabled)
{
	HAL_GPIO_WritePin(BT_RST_GPIO_Port, BT_RST_Pin, is_enabled);
}

/**
 * @brief BT121 boot mode control
 * @params
 * mode: 0 - boot mode disabled, 1 - boot mode enabled
 * @return
 * 0 - boot mode started, 1 - boot mode not started, 2 - boot state wasn't changed
 */
static uint8_t BT121_BootModeCtrl(uint8_t mode)
{
	uint8_t ret;
	uint8_t auto_br_byte = 0x7F;
	uint8_t result = HAL_OK;
	uint16_t rxLen = 0;
	uint8_t bt_ack_state = 0;

	if(is_boot_enabled != mode)
	{
		is_boot_enabled = mode;
		// reconfigure UART interface
		ret = BT121_ReconfigureUART(&huart3, mode);
		if(ret != HAL_OK)
		{
			return ret;
		}

		BT_RST_GPIO_Port->BSRR = BT_RST_Pin<<16; //	reset BT module
		HAL_GPIO_WritePin(BT_BOOT_GPIO_Port, BT_BOOT_Pin, mode); // set BOOT0 pin state
		HAL_Delay(5);
		BT_RST_GPIO_Port->BSRR = BT_RST_Pin; // release BT reset state
		// wait bootloader start
		HAL_Delay(20);

		if(mode) // if boot mode enabled
		{
			// send auto baudrate set byte
			ret = HAL_UART_Transmit(&huart3, &auto_br_byte, 1, 1000);
			if(ret != HAL_OK)
			{
				return HAL_ERROR;
			}
			// wait ACK answer from BT module
			ret = HAL_UARTEx_ReceiveToIdle(&huart3, &bt_ack_state, 1, &rxLen, 1000);
			if(bt_ack_state == ACK)
			{
				result = HAL_OK;
			}
			else
			{
				result = HAL_ERROR;
			}
		}
		else
		{
			// start data reception
			HAL_UARTEx_ReceiveToIdle_IT(&huart3, data_buffer, sizeof(data_buffer));
			result = HAL_OK;
		}
	}
	else
	{
		result = HAL_BUSY;
	}
	return result;
}

/**
 * @brief Write command to BT121 in  boot mode
 * @params
 * cmd: command byte
 * @return
 * 0 - command was sent successfully, 1 - command wasn't sent
 */
static uint8_t BT121_WriteBootCmd(uint8_t cmd)
{
	uint8_t ret;
	uint8_t bt_ack_state = 0;
	uint8_t command_data[2] = {cmd, cmd^0xFF};
	uint16_t rxLen = 0;
	// try 3 attempts to send command
	for(uint8_t i = 0; i < 3; i++)
	{
		ret = HAL_UART_Transmit(&huart3, command_data, 2, 1000);
		if(ret != HAL_OK)
		{
			HAL_UART_AbortTransmit(&huart3);
			continue;
		}

		// wait ACK answer from BT module
		ret = HAL_UARTEx_ReceiveToIdle(&huart3, &bt_ack_state, 1, &rxLen, 1000);
		if(ret != HAL_OK)
		{
			HAL_UART_AbortReceive(&huart3);
			continue;
		}
		if(bt_ack_state == ACK)
		{
			return HAL_OK;
		}
		else
		{
			continue;
		}
	}

	return HAL_ERROR;
}

/**
 * @brief Write data to BT121 in  boot mode
 * @params
 * pfirst_byte: if equals 0, transfer pbuff immediately
 * pbuff - data pointer
 * size - data array length
 * @return
 * 0 - data was sent successfully, 1 - data wasn't sent,
 */
static uint8_t BT121_WriteBootData(uint8_t *pfirst_byte, uint8_t *pbuff, uint16_t size)
{
	uint8_t ucval;
	uint8_t lrc = 0;
	uint8_t out_data[512] = {0};
	uint8_t ret;
	uint8_t bt_ack_state = 0;
	uint16_t rxLen = 0;
	uint8_t offset = 0;

	if(pfirst_byte != 0)
	{
	    ucval = *pfirst_byte;
	    lrc = lrc ^ ucval;
	    out_data[0] = ucval;
	    offset++;
	}
	// calculate CRC
	for (uint16_t i = 0; i < size; i++)
	{
		ucval = *pbuff++;
		lrc = lrc ^ ucval;
		out_data[i+offset] = ucval;
	}
	// store CRC value to out data array
	out_data[size+offset] = lrc;

	// transmit data to BT121 module
	ret = HAL_UART_Transmit(&huart3, out_data, size+1+offset, 1000);
	if(ret != HAL_OK)
	{
		return ret;
	}
	// wait ACK answer from BT module
	ret = HAL_UARTEx_ReceiveToIdle(&huart3, &bt_ack_state, 1, &rxLen, 100000); // long 100 sec delay for flash mass erase operation
	if(bt_ack_state == ACK)
	{
		return HAL_OK;
	}
	return ret;
}

/**
 * @brief Get BT121 bootloader version and commands list type
 * @params
 * pflash_erase_type: commands list
 * @return
 * 0 - operation is successful, 1 - operation is failed
 */
static uint8_t BT121_GetCmdsList(FLASH_ERASE_TYPE *pflash_erase_type)
{
	uint8_t res, ret;
	uint16_t size = 0;
	uint16_t rxLen = 0;
	uint8_t input_data[256] = {0};

	res = BT121_WriteBootCmd(BOOT_CMD_GET_CMD);
	if(res != HAL_OK)
		return res;
	// get command result
	ret = HAL_UARTEx_ReceiveToIdle(&huart3, input_data, 512, &rxLen, 1000);
	if(ret != HAL_OK)
	{
		return ret;
	}
	size = input_data[0] + 2; // +1B data +1B ACK/NACK
	// if received data size is incorrect
	if(rxLen < size)
	{
		return HAL_ERROR;
	}
	// if ACK isn't received
	if(input_data[rxLen-1] != ACK)
	{
		return HAL_ERROR;
	}
	// get commands list type
	for (uint16_t i = 1; i < (rxLen - 1); i++)
	{
		if (input_data[i] == 0x43)
			*pflash_erase_type = FLASH_ERASE_OLD;
		else if (input_data[i] == 0x44)
			*pflash_erase_type = FLASH_ERASE_EXT;
	}
	return HAL_OK;
}

/**
 * @brief Reconfigure UART in depend from boot of normal mode
 * @params
 * huart: UART HAL structure init handle
 * boot_mode: 0 - normal mode; 1 - boot mode
 * @return
 * 0 - operation is successful, 1 - operation is failed
 */
static uint8_t BT121_ReconfigureUART(UART_HandleTypeDef* huart, uint8_t boot_mode)
{
	uint8_t ret;
	// abort current UART receive operation
	ret = HAL_UART_AbortReceive_IT(&huart3);
	if(ret != HAL_OK)
	{
		return ret;
	}
	HAL_UART_DeInit(huart); // de-init UART

	huart->Instance = USART2;
	huart->Init.StopBits = UART_STOPBITS_1;
	huart->Init.Mode = UART_MODE_TX_RX;
	huart->Init.OverSampling = UART_OVERSAMPLING_16;
	// fill init handle structure
	if(boot_mode) // is boot mode
	{
	  huart->Init.BaudRate = 115200;
	  huart->Init.WordLength = UART_WORDLENGTH_9B;
	  huart->Init.Parity = UART_PARITY_EVEN;
	  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	}
	else // is normal mode
	{
	  huart->Init.BaudRate = 115200;
	  huart->Init.WordLength = UART_WORDLENGTH_8B;
	  huart->Init.Parity = UART_PARITY_NONE;
	  huart->Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
	}
	if (HAL_UART_Init(huart) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

/**
 * @brief Erase flash of BT121 module
 * @return
 * 0 - operation is successful, 1 - operation is failed,
 */
static uint8_t BT121_FlashErase(void)
{
	uint8_t res;
	uint8_t cmd;
	FLASH_ERASE_TYPE flash_erase_type;
	uint16_t size;
	uint8_t buff[2] = {0xFF, 0xFF}; // mass erase

	res = BT121_GetCmdsList(&flash_erase_type);
	if(res != HAL_OK)
		return res;

	if (flash_erase_type == FLASH_ERASE_EXT)
	{
		cmd = BOOT_CMD_ERASE_EX;
		size = 2;
	}
	else if (flash_erase_type == FLASH_ERASE_OLD)
	{
		cmd = BOOT_CMD_ERASE;
		size = 1;
	}
	else
		return HAL_ERROR;

	res = BT121_WriteBootCmd(cmd);
	if (res != HAL_OK)
		return res;

	res = BT121_WriteBootData(0, buff, size);
	if (res != HAL_OK)
		return res;

	return HAL_OK;
}

/**
 * @brief Write data to flash memory of BT121 module
 * flash_addr - start address
 * data - data buffer pointer
 * size - data buffer size
 * @return
 * 0 - operation is successful, 1 - operation is failed
 */
static uint8_t BT121_FlashWrite(uint32_t flash_addr, uint8_t* data, uint16_t size)
{
	uint8_t buff[4];
	uint8_t res;
	uint8_t sze;
	// write command
	res = BT121_WriteBootCmd(BOOT_CMD_WR_MEM);
	if(res != HAL_OK)
	{
		return BT_WRITE_BOOT_CMD_ERROR;
	}

	// write start flash address
	buff[0] = (int8_t)(flash_addr >> 24);
	buff[1] = (int8_t)(flash_addr >> 16);
	buff[2] = (int8_t)(flash_addr >> 8);
	buff[3] = (int8_t)(flash_addr);

	res = BT121_WriteBootData(0, buff, 4);
	if(res != HAL_OK)
		return BT_WRITE_ADDRESS_ERROR;

	// write data buffer to BT121 flash memory
	sze = (uint8_t)(size - 1);
	res = BT121_WriteBootData(&sze, data, (uint16_t)size);
	if(res != HAL_OK)
		return BT_WRITE_BOOT_DATA_ERROR;

	return HAL_OK;
}

/**
 * @brief Verify written data to flash memory of BT121 module
 * flash_addr - start address
 * check_data - data buffer for check pointer
 * size - data buffer size
 * @return
 * 0 - operation is successful, 1 - operation is failed
 */
static uint8_t BT121_FlashVerify(uint32_t flash_addr, uint8_t* check_data, uint16_t check_data_size)
{
	uint8_t buff[4];
	uint8_t res;
	uint16_t rxLen = 0;
	uint8_t input_data[256] = {0};
	// write command
	res = BT121_WriteBootCmd(BOOT_CMD_RD_MEM);
	if(res != HAL_OK)
		return BT_VERIFY_BOOT_CMD_ERROR;

	// write start flash address
	buff[0] = (int8_t)(flash_addr >> 24);
	buff[1] = (int8_t)(flash_addr >> 16);
	buff[2] = (int8_t)(flash_addr >> 8);
	buff[3] = (int8_t)(flash_addr);

	res = BT121_WriteBootData(0, buff, 4);
	if(res != HAL_OK)
		return BT_VERIFY_ADDRESS_ERROR;

	// write read data size
	res = BT121_WriteBootCmd((uint8_t)(check_data_size-1));
	if(res != HAL_OK)
		return BT_VERIFY_BOOT_CMD_ERROR;

	// read data from flash memory
	res = HAL_UARTEx_ReceiveToIdle(&huart3, input_data, check_data_size, &rxLen, 10000);
	if(res != HAL_OK || rxLen != check_data_size)
	{
		return BT_VERIFY_BOOT_READ_DATA_ERROR;
	}

	// verify read flash data
	for(uint8_t i = 0; i < check_data_size; i++)
	{
		if(check_data[i] != input_data[i])
		{
			return BT_FLASH_DATA_UNMATCH;
		}
	}
	return HAL_OK;
}

/**
 * @brief Send input HID report via BT121
 * @params
 * report_id: id number of report
 * report_data: report data array
 * report_length: report data array length
 */
static void BT121_SendInputReport(uint8_t report_id, uint8_t* report_data, uint8_t report_length)
{
	uint8_t temp_data[10] = {0};
	uint8_t ret;

	temp_data[0] = BT_INPUT_REPORT_DATA_HEADER; // input report data header
	temp_data[1] = report_id;

	memcpy(temp_data+2, report_data, (report_length <= 8) ? (report_length) : (8));

	ret = HAL_UART_Transmit(&huart3, temp_data, (report_length <= 8) ? (report_length + 2) : (10), 1000);
	if(ret != HAL_OK)
	{
		Error_Handler();
		return;
	}

}

/**
 * @brief Get HID endpoint connection state
 * @return
 * 0 - HID endpoint disconnected, 1 - HID endpoint disconnected
 */
static uint8_t BT121_IsHID_EndpointConnected(void)
{
	return is_hid_connected;
}

/**
 * @brief Delete all bonding information from Persistent Store of BT121
 */
static void BT121_DeleteBonding(void)
{
	uint8_t cmd_data[2] = {0};
	uint8_t ret;

	cmd_data[0] = BT_COMMAND_DATA_HEADER; // command data header
	cmd_data[1] = BT_DELETE_BONDING_CMD;  // delete bonding command

	ret = HAL_UART_Transmit(&huart3, cmd_data, 2, 1000);
	if(ret != HAL_OK)
	{
		Error_Handler();
		return;
	}
}

/**
 * @brief Get BT mode boot mode state
 * @return
 * 0 - boot mode disabled, 1 - boot mode enabled
 */
static uint8_t BT121_IsBootModeEnabled(void)
{
	return is_boot_enabled;
}


/**
 * @brief Process output HID report from BT121
 * @params
 * report_id: id number of report
 * report_data: report data array
 * report_length: report data array length
 */
static void BT121_ProcessOutputReport(uint8_t report_id, uint8_t* report_data, uint8_t report_length)
{
//TODO: add output reports processing
}

/**
 * @brief Process input data received via UART interface from BT121
*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	uint16_t offset = 0;
	// If first byte is zero skip it to avoid possible desync due to inherent uart framing error on module reset
	offset = (data_buffer[0] == 0) ? (1) : (0);

	switch (data_buffer[offset])
	{
		// connection status
		case BT_CONNECTION_STATUS_HEADER:
			switch(data_buffer[offset+1])
			{
				case BT_NOT_CONNECTED:
					is_hid_connected = 0;
					break;

				case BT_IS_CONNECTING:
					break;

				case BT_IS_CONNECTED:
					is_hid_connected = 1;
					break;
			}
			break;

		// output report data
		case BT_OUTPUT_REPORT_DATA_HEADER:
			BT121_ProcessOutputReport(data_buffer[offset+1], data_buffer+offset+2, Size - 2 - offset);
			break;
	}

    HAL_UARTEx_ReceiveToIdle_IT(&huart3, data_buffer, sizeof(data_buffer)); // prepare to receive next data
}

/**
 * @brief Transfer end event via UART interface to BT121
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	is_transfer_ended = 1;
}
