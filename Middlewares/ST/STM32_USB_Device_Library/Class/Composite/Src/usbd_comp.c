/**
  *******************************************************************************
  *
  * @file    usbd_comp.c
  * @brief   USB composite device driver
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


/* Includes ------------------------------------------------------------------*/
#include "usbd_comp.h"
#include "usbd_audio.h"
#include "usbd_audio_if.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"
#include "usbd_ctlreq.h"

#include <stdio.h>
#include "main.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
// clang-format off

/* Private macro -------------------------------------------------------------*/

/* Private function prototypes------------------------------------------------*/
static uint8_t USBD_COMP_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t USBD_COMP_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t USBD_COMP_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);

static uint8_t* USBD_COMP_GetCfgDesc (uint16_t *length);

static uint8_t* USBD_COMP_GetDeviceQualifierDesc (uint16_t *length);

static uint8_t USBD_COMP_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t USBD_COMP_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t USBD_COMP_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t USBD_COMP_EP0_TxReady (USBD_HandleTypeDef *pdev);

static uint8_t USBD_COMP_SOF (USBD_HandleTypeDef *pdev);

static uint8_t USBD_COMP_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t USBD_COMP_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum);

/* Private variables ---------------------------------------------------------*/

USBD_ClassTypeDef  USBD_COMP =
{
  USBD_COMP_Init,
  USBD_COMP_DeInit,
  USBD_COMP_Setup,
  USBD_COMP_EP0_TxReady,
  USBD_COMP_EP0_RxReady,
  USBD_COMP_DataIn,
  USBD_COMP_DataOut,
  USBD_COMP_SOF,
  USBD_COMP_IsoINIncomplete,
  USBD_COMP_IsoOutIncomplete,
  USBD_COMP_GetCfgDesc,
  USBD_COMP_GetCfgDesc,
  USBD_COMP_GetCfgDesc,
  USBD_COMP_GetDeviceQualifierDesc,
};

extern USBD_HandleTypeDef hUsbDeviceFS;

USBD_COMP_ItfTypeDef USBD_COMP_fops_FS;
USBD_ClassCompInfo comp_dev [CLASS_NUM];

extern uint8_t USBD_AUDIO_CfgDesc[USB_AUDIO_CONFIG_DESC_SIZ];
extern uint8_t USBD_CUSTOM_HID_CfgFSDesc[USB_CUSTOM_HID_CONFIG_DESC_SIZ];

/* USB COMP device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMP_CfgDesc[USB_COMP_CONFIG_DESC_SIZ] __ALIGN_END;

__ALIGN_BEGIN static uint8_t USBD_COMP_DevCfgDesc[9] __ALIGN_END = {
	0x09,                                 /* bLength */
	USB_DESC_TYPE_CONFIGURATION,          /* bDescriptorType */
	LOBYTE(USB_COMP_CONFIG_DESC_SIZ),    /* wTotalLength  232 bytes*/
	HIBYTE(USB_COMP_CONFIG_DESC_SIZ),
	USBD_MAX_NUM_INTERFACES,    //NumInterfaces:4
	0x01,    //ConfigurationValue
	0x00,    //Configuration String
	0x80,    //Attributes:Bus Power
	0xFA,    //MaxPower = 0xfa*2ma
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMP_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END=
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/* Private function prototypes -----------------------------------------------*/


/* Private user code ---------------------------------------------------------*/


/* External variables --------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

static inline void switchToClass (USBD_HandleTypeDef *pdev, USBD_ClassCompInfo *class)
{
  pdev->pClassData = class->classData;
  pdev->pUserData  = class->userData;
}

static inline void saveClass (USBD_HandleTypeDef *pdev, USBD_ClassCompInfo *class)
{
  class->classData = pdev->pClassData;
  class->userData  = pdev->pUserData;
}

/**
 * @brief  USBD_COMP_Init
 *         Initialize the COMP interface
 * @param  pdev:   device instance
 * @param  cfgidx: configuration index
 * @retval status
 */

static uint8_t USBD_COMP_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  uint8_t retval = USBD_OK;

  comp_dev[UAC].class    = &USBD_AUDIO;
  comp_dev[UAC].userData = &USBD_AUDIO_fops_FS;
  comp_dev[UAC].ctrlIf   = AUDIO_CTRL_IF;
  comp_dev[UAC].minIf    = AUDIO_CTRL_IF;
  comp_dev[UAC].maxIf    = AUDIO_OUT_IF;

  switchToClass (pdev, &comp_dev[UAC]);
  retval = comp_dev[UAC].class->Init (pdev, cfgidx);
  saveClass (pdev, &comp_dev[UAC]);

  comp_dev[HID].class    = &USBD_CUSTOM_HID;
  comp_dev[HID].userData = &USBD_CustomHID_fops_FS;
  comp_dev[HID].ctrlIf   = HID_CTRL_IF;
  comp_dev[HID].minIf    = HID_CTRL_IF;
  comp_dev[HID].maxIf    = HID_CTRL_IF;

  switchToClass (pdev, &comp_dev[HID]);
  retval = comp_dev[HID].class->Init (pdev, cfgidx);
  saveClass (pdev, &comp_dev[HID]);

  return retval;
}

/**
 * @brief  USBD_COMP_DeInit
 *         DeInitialize the COMP layer
 * @param  pdev:   device instance
 * @param  cfgidx: configuration index
 * @retval status
 */

static uint8_t USBD_COMP_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  uint8_t retval = USBD_OK;

  for (uint8_t i = 0U; i < CLASS_NUM; i++)
  {
    switchToClass (pdev, &comp_dev[i]);
    retval = comp_dev[i].class->DeInit (pdev, cfgidx);
    saveClass (pdev, &comp_dev[i]);
  }
  return retval;
}

/**
 * @brief  USBD_COMP_Setup
 *         Handle the AUDIO specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */

static uint8_t USBD_COMP_Setup (USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  uint8_t  retval = USBD_OK;
  uint8_t  done = 0U;
  uint8_t  i;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest)
      {
        case AUDIO_REQ_GET_CUR:
        case AUDIO_REQ_SET_CUR:
        case AUDIO_REQ_GET_MAX:
        case AUDIO_REQ_GET_MIN:
        case AUDIO_REQ_GET_RES:
          switchToClass (pdev, &comp_dev[UAC]);
          retval = comp_dev[UAC].class->Setup (pdev, req);
          if (!retval) done = 1U;
          break;

        case CUSTOM_HID_REQ_SET_PROTOCOL:
        case CUSTOM_HID_REQ_GET_PROTOCOL:
        case CUSTOM_HID_REQ_SET_IDLE:
        case CUSTOM_HID_REQ_GET_IDLE:
        case CUSTOM_HID_REQ_SET_REPORT:
          switchToClass (pdev, &comp_dev[HID]);
          retval = comp_dev[HID].class->Setup (pdev, req);
          if (!retval) done = 1U;
          break;
      }

      if (done != 1U)
      {
        USBD_CtlError (pdev, req);
        retval = USBD_FAIL;
      }

      break;

    case USB_REQ_TYPE_STANDARD:

      switch (req->bRequest)
      {
      	case USB_REQ_GET_STATUS:
        case USB_REQ_GET_DESCRIPTOR:
        case USB_REQ_GET_INTERFACE:
        case USB_REQ_SET_INTERFACE:
			for (i = 0U; i < CLASS_NUM; i++)
			{
			  if ((req->wIndex >= comp_dev[i].minIf) && (req->wIndex <= comp_dev[i].maxIf))
			  {
				switchToClass (pdev, &comp_dev[i]);
				retval = comp_dev[i].class->Setup (pdev, req);
				done = 1U;
				break;
			  }
			}
          break;

        default:
          USBD_CtlError (pdev, req);
          retval = USBD_FAIL;
      }
  }
  return retval;
}

/**
 * @brief  USBD_COMP_DataIn
 *         handle data IN Stage
 * @param  pdev:  device instance
 * @param  epnum: endpoint index
 * @retval status
 */

static uint8_t  USBD_COMP_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  uint8_t retval = USBD_OK;
  uint8_t i;

  if (epnum == (CUSTOM_HID_EPIN_ADDR & 0x7F)) i = HID;
  if (epnum == (AUDIO_IN_EP & 0x7F) || epnum == (SYNC_IN_EP & 0x7F)) i = UAC;

  if (comp_dev[i].class->DataIn != NULL)
  {
    switchToClass (pdev, &comp_dev[i]);
    retval = comp_dev[i].class->DataIn (pdev, epnum);
  }

  return retval;
}

/**
 * @brief  USBD_COMP_DataOut
 *         handle data OUT Stage
 * @param  pdev:  device instance
 * @param  epnum: endpoint index
 * @retval status
 */

static uint8_t  USBD_COMP_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  uint8_t retval = USBD_OK;
  uint8_t i;

  if (epnum == CUSTOM_HID_EPOUT_ADDR) i = HID;
  if (epnum == AUDIO_OUT_EP) i = UAC;

  if (comp_dev[i].class->DataOut != NULL)
  {
    switchToClass (pdev, &comp_dev[i]);
    retval = comp_dev[i].class->DataOut (pdev, epnum);
  }

  return retval;
}

/**
 * @brief  USBD_COMP_SOF
 *         handle SOF event
 * @param  pdev: device instance
 * @retval status
 */

static uint8_t  USBD_COMP_SOF (USBD_HandleTypeDef *pdev)
{
  uint8_t retval = USBD_OK;

  for (uint8_t i = 0U; i < CLASS_NUM; i++)
  {
    if (comp_dev[i].class->SOF != NULL)
    {
      switchToClass (pdev, &comp_dev[i]);
      retval = comp_dev[i].class->SOF (pdev);
    }
  }
  return retval;
}

/**
 * @brief  USBD_COMP_EP0_RxReady
 *         handle EP0 Rx Ready event
 * @param  pdev: device instance
 * @retval status
 */

static uint8_t  USBD_COMP_EP0_RxReady (USBD_HandleTypeDef *pdev)
{
  uint8_t retval = USBD_OK;

  for (uint8_t i = 0U; i < CLASS_NUM; i++)
  {
    if (comp_dev[i].class->EP0_RxReady != NULL)
    {
      switchToClass (pdev, &comp_dev[i]);
      retval = comp_dev[i].class->EP0_RxReady (pdev);
    }
  }
  return retval;
}

/**
  * @brief  USBD_COMP_EP0_TxReady
  *         handle EP0 Tx Ready event
  * @param  pdev: device instance
  * @retval status
  */

static uint8_t  USBD_COMP_EP0_TxReady (USBD_HandleTypeDef *pdev)
{
  /* Only OUT control data are processed */
  return USBD_OK;
}

/**
  * @brief  USBD_COMP_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev:  device instance
  * @param  epnum: endpoint index
  * @retval status
  */

static uint8_t  USBD_COMP_IsoINIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  uint8_t retval = USBD_OK;

  if (comp_dev[UAC].class->IsoINIncomplete != NULL)
  {
		switchToClass (pdev, &comp_dev[UAC]);
		retval = comp_dev[UAC].class->IsoINIncomplete (pdev, epnum);
  }

  return retval;
}

/**
  * @brief  USBD_COMP_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev:  device instance
  * @param  epnum: endpoint index
  * @retval status
  */

static uint8_t  USBD_COMP_IsoOutIncomplete (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  uint8_t retval = USBD_OK;

  if (epnum == AUDIO_OUT_EP)
  {
	  if (comp_dev[UAC].class->IsoOUTIncomplete != NULL)
	  {
			switchToClass (pdev, &comp_dev[UAC]);
			retval = comp_dev[UAC].class->IsoOUTIncomplete (pdev, epnum);
	  }
  }

  return retval;
}

/**
 * @brief  USBD_COMP_GetCfgDesc
 *         return Configuration descriptor
 * @param  length: pointer data length
 * @retval pointer to descriptor buffer
 */

static uint8_t* USBD_COMP_GetCfgDesc (uint16_t *length)
{
	// fill composite device configuration descriptor
	memcpy(USBD_COMP_CfgDesc, USBD_COMP_DevCfgDesc, 9); // copy configuration descriptor
	memcpy(USBD_COMP_CfgDesc + 9, USBD_AUDIO_CfgDesc + 9, USB_AUDIO_CONFIG_DESC_SIZ - 9); // copy audio configuration descriptor
	memcpy(USBD_COMP_CfgDesc + USB_AUDIO_CONFIG_DESC_SIZ, USBD_CUSTOM_HID_CfgFSDesc + 9, USB_CUSTOM_HID_CONFIG_DESC_SIZ - 9); // copy custom HID configuration descriptor

	*length = sizeof (USBD_COMP_CfgDesc);
	return USBD_COMP_CfgDesc;
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length: pointer data length
* @retval pointer to descriptor buffer
*/

static uint8_t* USBD_COMP_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_COMP_DeviceQualifierDesc);
  return USBD_COMP_DeviceQualifierDesc;
}

/**
 * @brief  USBD_COMP_RegisterInterface
 * @param  fops: COMP interface callback
 * @retval status
 */

uint8_t USBD_COMP_RegisterInterface (USBD_HandleTypeDef   *pdev,
                                     USBD_COMP_ItfTypeDef *fops)
{
  if (fops != NULL)
  {
    pdev->pUserData = fops;
  }
  return USBD_OK;
}


/**
  * @brief  HID_Transmit_FS
  *         Data to send over USB IN endpoint are sent over HID interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */

uint8_t USBD_COMP_HID_SendReport_FS (uint8_t* Buf, uint16_t Len)
{
  switchToClass (&hUsbDeviceFS, &comp_dev[HID]);
  return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, Buf, Len);
}

void USBD_COMP_AUDIO_UpdateBuffers(AUDIO_OffsetTypeDef offset)
{
	switchToClass (&hUsbDeviceFS, &comp_dev[UAC]);
	USBD_AUDIO_UpdateBuffers(&hUsbDeviceFS, offset);
}

/****END OF FILE****/
