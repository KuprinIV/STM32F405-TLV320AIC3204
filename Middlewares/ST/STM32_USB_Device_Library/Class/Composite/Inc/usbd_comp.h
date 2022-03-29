/**
  *******************************************************************************
  *
  * @file    usbd_comp.h
  * @brief   header file for the usbd_comp.c file
  * @version v1.0
  * @date    18.07.2020
  * @author  Dmitrii Rudnev
  *
  *******************************************************************************
  * Copyrigh &copy; 2020 Selenite Project. All rights reserved.
  *
  * This software component is licensed under [BSD 3-Clause license]
  * (http://opensource.org/licenses/BSD-3-Clause/), the "License".<br>
  * You may not use this file except in compliance with the License.
  *******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_COMP_H_
#define __USBD_COMP_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_ioreq.h"
#include "usbd_desc.h"
#include "stm32f4xx_hal.h"
#include "usbd_audio.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	void *ptr;
} USBD_COMP_ItfTypeDef;

typedef struct
{
  USBD_ClassTypeDef *class;
  void *classData;
  void *userData;
  uint16_t ctrlIf;
  uint16_t minIf;
  uint16_t maxIf;
} USBD_ClassCompInfo;


/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variables---------------------------------------------------------*/
extern USBD_ClassTypeDef    USBD_COMP;
extern USBD_COMP_ItfTypeDef USBD_COMP_fops_FS;

/* Exported functions prototypes ---------------------------------------------*/
uint8_t USBD_COMP_RegisterInterface (USBD_HandleTypeDef *pdev, USBD_COMP_ItfTypeDef *fops);
uint8_t USBD_COMP_HID_SendReport_FS(uint8_t* Buf, uint16_t  Len);
void USBD_COMP_AUDIO_UpdateOutBuffer(void);
void USBD_COMP_AUDIO_UpdateInBuffer(void);

/* Private defines -----------------------------------------------------------*/
#define CLASS_NUM                    2U
#define UAC                          0U
#define HID                          1U
#define USBD_COMP_CLASS              &USBD_COMP

#ifdef USE_SYNC_EP
#define USB_COMP_CONFIG_DESC_SIZ     233U
#else
#define USB_COMP_CONFIG_DESC_SIZ     224U
#endif

#define HID_CTRL_IF					 3U

#endif /* ST_STM32_USB_DEVICE_LIBRARY_CLASS_COMP_INC_USBD_COMP_H_ */
