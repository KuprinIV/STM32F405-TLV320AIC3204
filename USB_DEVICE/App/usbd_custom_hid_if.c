/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @version        : v1.0_Cube
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */
#include "usbd_comp.h"
#include "tlv320aic3204.h"
#include "bt121.h"
#include "keyboard.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @addtogroup USBD_CUSTOM_HID
  * @{
  */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions USBD_CUSTOM_HID_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Defines USBD_CUSTOM_HID_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Macros USBD_CUSTOM_HID_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Variables USBD_CUSTOM_HID_Private_Variables
  * @brief Private variables.
  * @{
  */

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  /* USER CODE BEGIN 0 */
	0x06, 0x00, 0xff,              // USAGE_PAGE (Vendor Defined Page 1)
	0x09, 0x01,                    // USAGE (Vendor Usage 1)
	0xa1, 0x01,                    // COLLECTION (Application)

	// keyboard state report (bytes: 0 - report ID (0x01), 1,2 - pressed key codes, 3 - delay MSB, 4 - delay LSB,  5 - joystick left X coordinate (-127...127),
	// 6 -  joystick left Y coordinate (-127...127), 7 - joystick right X coordinate (-127...127), 8 -  joystick right Y coordinate (-127...127),
	//	9 - battery charge state (0...100))
	0x09, 0x01,                    //   USAGE (Vendor Usage 1)
	0x85, 0x01,               	   //   REPORT_ID (1)
	0x95, 0x09,                    //   REPORT_COUNT (9)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x81, 0x82,                    //   INPUT (Data,Var,Abs)

	// get report from device (bytes: 0 - report ID (0x02), 1 - get report command: 0x01 - send report, other - no action)
	0x09, 0x01,                    //   USAGE (Vendor Usage 1)
	0x85, 0x02,               	   //   REPORT_ID (2)
	0x95, 0x01,                    //   REPORT_COUNT (1)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x91, 0x82,                    //   INPUT (Data,Var,Abs)

	// audio control report (bytes: 0 - report ID (0x03), 1 - interface direction (0x00 - out, 0x01 - in), 2 - output selection (0x00 - headphones,
	// 0x01 - loudspeakers), 3 - input selection (0x00 - IN1, 0x01 - IN3))
	0x09, 0x01,                    //   USAGE (Vendor Usage 1)
	0x85, 0x03,               	   //   REPORT_ID (3)
	0x95, 0x03,                    //   REPORT_COUNT (3)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x91, 0x82,                    //   OUTPUT (Data,Var,Abs)

	// LED control report (bytes: 0 - report ID (0x04), 1 - pulse length MSB, 2 - pulse length LSB, 3 - green color byte, 4 - red color byte,
	// 5 - blue color byte)
	0x09, 0x01,                    //   USAGE (Vendor Usage 1)
	0x85, 0x04,               	   //   REPORT_ID (4)
	0x95, 0x05,                    //   REPORT_COUNT (5)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x91, 0x82,                    //   OUTPUT (Data,Var,Abs)

	// misc control report (bytes: 0 - report ID (0x05), 1 - delay test control (0x00 - stop test, 0x01 - start test),
	// 2 - BT firmware update command (0x00 - exit from BT boot mode, 0x01 - enter to the BT boot mode),
	// 3 - enter to DFU boot mode (0x01), 4 - joysticks calibration mode control (0x00 - exit from calibration mode
	// 0x01 - enter to calibration mode)),
	0x09, 0x01,                    //   USAGE (Vendor Usage 1)
	0x85, 0x05,               	   //   REPORT_ID (5)
	0x95, 0x04,                    //   REPORT_COUNT (4)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x91, 0x82,                    //   OUTPUT (Data,Var,Abs)

	// BT firmware data report (bytes: 0 - report ID (0x06), 1...64 - BT firmware data)
	0x09, 0x01,                    //   USAGE (Vendor Usage 1)
	0x85, 0x06,               	   //   REPORT_ID (6)
	0x95, 0x40,                    //   REPORT_COUNT (64)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x91, 0x82,                    //   OUTPUT (Data,Var,Abs)

	// BT firmware packet write status (bytes: 0 - report ID (0x07), 1 - firmware packet transfer data state (0x00 - success, other - error))
	0x09, 0x01,                    //   USAGE (Vendor Usage 1)
	0x85, 0x07,               	   //   REPORT_ID (7)
	0x95, 0x01,                    //   REPORT_COUNT (1)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x81, 0x82,                    //   INPUT (Data,Var,Abs)

	// joysticks calibration data (bytes: 0 - report ID (0x08); 1,2 - joystick left H-axis max value; 3,4 - joystick left H-axis min value;
	// 5,6 - joystick left H-axis zero value; 7,8 - joystick left Y-axis max value; 9,10 - joystick left Y-axis min value;
	// 11,12 - joystick left Y-axis zero value; 13,14 - joystick right H-axis max value; 15,16 - joystick right H-axis min value;
	// 17,18 - joystick right H-axis zero value; 19,20 - joystick right Y-axis max value; 21,22 - joystick right Y-axis min value;
	// 23,24 - joystick right Y-axis zero value)
	0x09, 0x01,                    //   USAGE (Vendor Usage 1)
	0x85, 0x08,               	   //   REPORT_ID (8)
	0x95, 0x18,                    //   REPORT_COUNT (24)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x91, 0x82,                    //   OUTPUT (Data,Var,Abs)

	// joysticks position raw data (bytes: 0 - report ID (0x09), 1 - joystick left H-axis pos MSB, 2 - joystick left H-axis pos LSB,
	// 3 - joystick left Y-axis pos MSB, 4 - joystick left Y-axis pos LSB, 5 - joystick right H-axis pos MSB, 6 - joystick right H-axis pos LSB,
	// 7 - joystick right Y-axis pos MSB, 8 - joystick right Y-axis pos LSB))
	0x09, 0x01,                    //   USAGE (Vendor Usage 1)
	0x85, 0x09,               	   //   REPORT_ID (9)
	0x95, 0x08,                    //   REPORT_COUNT (8)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x81, 0x82,                    //   INPUT (Data,Var,Abs)
  /* USER CODE END 0 */
  0xC0    /*     END_COLLECTION	             */
};

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
  * @brief Public variables.
  * @{
  */

/* USER CODE BEGIN EXPORTED_VARIABLES */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END EXPORTED_VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t report_num);
//static int8_t USBD_CUSTOM_HID_SendReport_FS(uint8_t *report, uint16_t len);

/**
  * @}
  */

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS =
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS,
};

/** @defgroup USBD_CUSTOM_HID_Private_Functions USBD_CUSTOM_HID_Private_Functions
  * @brief Private functions.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  DeInitializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Manage the CUSTOM HID class events
  * @param  event_idx: Event index
  * @param  state: Event state
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t report_num)
{
  /* USER CODE BEGIN 6 */
	uint8_t inputData[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE]= {0};
	uint16_t pulse_length = 0;
	uint32_t color_grb = 0;
	uint16_t joystickLeftCalibData[6] = {0};
	uint16_t joystickRightCalibData[6] = {0};

	USBD_CUSTOM_HID_HandleTypeDef  *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData;
	memcpy(inputData, hhid->Report_buf, USBD_CUSTOMHID_OUTREPORT_BUF_SIZE);

	switch(report_num)
	{
		case 2:
			kbState->isGetReportCmdReceived = (inputData[1] & 0x01);
			break;

		case 3:
			tlv320aic3204_drv->SelectOutput(inputData[2] & 0x01); // select codec's outputs
			tlv320aic3204_drv->SelectInput(inputData[3] & 0x01); // select codec's input
			break;

		case 4:
			pulse_length = (inputData[1]<<8)|inputData[2]; // LED pulse length in ms
			color_grb = (inputData[3]<<16)|(inputData[4]<<8)|inputData[5]; // LED color
			kbState->SetFrontLedColor(pulse_length, color_grb); // set front LED color
			break;
		
		case 5:
			kbState->isDelayMeasureTestEnabled = (inputData[1] & 0x01); // delay test control
			// BT boot mode control
			if((inputData[2] & 0x01) == 1)
			{
				bt121_fw_update->startBTBootMode = 1;
			}
			else
			{
				bt121_fw_update->stopBTBootMode = 1;
			}
			// enter into DFU mode
			kbState->isDfuModeEnabled = (inputData[3] & 0x01);
			// joysticks calibration mode control
			kbState->JoysticksCalibrationModeControl(inputData[4] & 0x01);
			break;

		case 6:
			bt121_fw_update->isBtFwUpdateStarted = 1;
			memcpy(bt121_fw_update->btFwPacket64b, &inputData[1], USBD_CUSTOMHID_OUTREPORT_BUF_SIZE-1);
			bt121_fw_update->isBtReadyToReceiveNextPacket = 0;
			break;

		case 8:
			memcpy(joystickLeftCalibData, &inputData[1], sizeof(joystickLeftCalibData));
			memcpy(joystickRightCalibData, &inputData[sizeof(joystickLeftCalibData)+1], sizeof(joystickRightCalibData));
			kbState->SaveJoysticksCalibrationData(joystickLeftCalibData, joystickRightCalibData);
			break;

		default:
			break;
	}
  return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */
/**
  * @brief  Send the report to the Host
  * @param  report: The report to be sent
  * @param  len: The report length
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */

//static int8_t USBD_CUSTOM_HID_SendReport_FS(uint8_t *report, uint16_t len)
//{
//	COMP_HID_Transmit_FS(report, len);
//	return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len);
//}

/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

