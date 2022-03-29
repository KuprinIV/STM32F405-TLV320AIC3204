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


/* USB COMP device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMP_CfgDesc[USB_COMP_CONFIG_DESC_SIZ] __ALIGN_END =
{
    /* Configuration 1 */
	0x09,                                 /* bLength */
	USB_DESC_TYPE_CONFIGURATION,          /* bDescriptorType */
	LOBYTE(USB_COMP_CONFIG_DESC_SIZ),    /* wTotalLength  232 bytes*/
	HIBYTE(USB_COMP_CONFIG_DESC_SIZ),
	USBD_MAX_NUM_INTERFACES,    //NumInterfaces:4
	0x01,    //ConfigurationValue
	0x00,    //Configuration String
	0x80,    //Attributes:Bus Power
	0xFA,    //MaxPower = 0xfa*2ma
    /* 09 bytes */
   /**********************Audio Interface Descriptor(No.0):0x04**********************/
	//第一个接口，控制接口
	AUDIO_INTERFACE_DESC_SIZE,    //Length
	USB_DESC_TYPE_INTERFACE,    //DescriptorType:Inerface
	AUDIO_CTRL_IF,    //InterfaceNum:0
	0x00,    //AlternateSetting:0
	0x00,    //NumEndpoint:0
	USB_DEVICE_CLASS_AUDIO,    //InterfaceClass:audio
	AUDIO_SUBCLASS_AUDIOCONTROL,    //InterfaceSubClass:audio ctl
	AUDIO_PROTOCOL_UNDEFINED,    //InterfaceProtocol
	0x00,    //Interface String

	/*******************AC Header of Interface Descriptor:0x24 0x01*************************/
	AUDIO_INTERFACE_DESC_SIZE+1,    //Length
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,    //DescriptorType:audio interface descriptor
	AUDIO_CONTROL_HEADER,    //DescriptorSubType:audio control header
	0x00,
	0x01,    //bcdADC:audio Device Class v1.00
	0x46,
	0x00,    //TotalLength:0x0048 （这个长度应该是特性单元跟Terminal描述符的总长度，包括自己 Total size of class specific descriptors.）
	0x02,    //InCollection:2 AudioStreaming interface
	AUDIO_IN_IF,    //InterfaceNr(2) - AS #1 id AudioStreaming interface 2 belongs to this AudioControl interface
	AUDIO_OUT_IF,    //InterfaceNr(1) - AS #2 id AudioStreaming interface 1 belongs to this AudioControl interface


	/******************* AC Specific type of Input Terminal:0x24 0x02*************************/
	//ID1
	AUDIO_INPUT_TERMINAL_DESC_SIZE,    //Length
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,    //DescriptorType:audio interface descriptor
	AUDIO_CONTROL_INPUT_TERMINAL,    //DescriptorSubType:Input Terminal
	0x01,    //TerminalID:0x01
	0x01,
	0x02,    //TerminalType:USB Microphone
	0x00,    //AssocTerminal
	0x01,    //NrChannels:2 channel
	0x01,
	0x00,    //ChannelConfig:Left Front,Right Front,
	0x00,    //ChannelName String
	0x00,    //Terminal String

	/*******************Audio Class Specific type of Feature Unit:0x24 0x06*************************/
	//ID2
	0x09,    //Length
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,    //DescriptorType:audio interface descriptor
	AUDIO_CONTROL_FEATURE_UNIT,    //DescriptorSubType:Audio Feature Unit
	AUDIO_OUT_STREAMING_CTRL,    //UnitID:0x02
	0x01,    //SourceID:1 #Microphone IT
	0x01,    //ControlSize:1 byte
	AUDIO_CONTROL_MUTE,    //Controls:Mute
	0x00,    //Controls(0):Volume
	0x00,    //Feature String

	/*******************Audio Class Specific type of Output Terminal:0x24 0x03*************************/
	//ID3
	0x09,    //Length
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,    //DescriptorType:audio interface descriptor
	AUDIO_CONTROL_OUTPUT_TERMINAL,    //DescriptorSubTYpe:Output Terminal
	0x03,    //TerminalID:0x03
	0x01,
	0x01,    //TerminalType:USB Streaming
	0x00,    //AssocTerminal:ID 0
	0x02,    //SourceID:2 #Feature UNIT   （ID 2作为控制microphone音量 - by ywj)
	0x00,    //Terminal String

	/******************* Audio Class Specific type of Input Terminal:0x24 0x02*************************/
	//ID4
	AUDIO_INPUT_TERMINAL_DESC_SIZE,    //Length
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,    //DescriptorType:audio interface descriptor
	AUDIO_CONTROL_INPUT_TERMINAL,    //DescriptorSubType:Input Terminal
	0x04,    //TerminalID:0x04
	0x01,
	0x01,    //TerminalType:USB Streaming
	0x00,    //AssocTerminal
	0x02,    //NrChannels:2 channel
	0x03,
	0x00,    //ChannelConfig:Left Front,Right Front,
	0x00,    //ChannelName String
	0x00,    //Terminal String

	/*******************Audio Class Specific type of Feature Unit:0x24 0x06*************************/
	//ID5
	0x09,    //Length
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,    //DescriptorType:audio interface descriptor
	AUDIO_CONTROL_FEATURE_UNIT,    //DescriptorSubType:Audio Feature Unit
	0x05,    //UnitID:0x05
	0x04,    //SourceID:4 #USB Streaming IT
	0x01,    //ControlSize:1 byte
	AUDIO_CONTROL_FEATURES,
	0x00,    //Controls(1):Volume
	0x00,    //Feature String

	/*******************Audio Class Specific type of Output Terminal:0x24 0x03*************************/
	//ID6
	0x09,    //Length
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,    //DescriptorType:audio interface descriptor
	AUDIO_CONTROL_OUTPUT_TERMINAL,    //DescriptorSubTYpe:Output Terminal
	0x06,    //TerminalID:0x06
	0x01,
	0x03,    //TerminalType:Speaker
	0x00,    //AssocTerminal:
	0x05,    //SourceID:5 #Feature UNIT   (ID 5作为控制speak音量 - by ywj)
	0x00,    //Terminal String

	/*****************Audio Recorder Interface descriptor(No.1):0x04***********************/
	//录音部分 - audioStreaming interface 1
	//Operational Alternate Setting 0
	0x09,    //Length
	USB_DESC_TYPE_INTERFACE,    //DescriptorType:Interface
	AUDIO_IN_IF,    //InterfaceNum:1
	0x00,    //AlternateSetting:0
	0x00,    //NumEndpoint:0
	USB_DEVICE_CLASS_AUDIO,    //InterfaceClass:audio
	AUDIO_SUBCLASS_AUDIOSTREAMING,    //InterfaceSubClass:audio streaming
	AUDIO_PROTOCOL_UNDEFINED,    //InterfaceProtocol
	0x00,    //Interface String

	/*****************Audio Recorder Interface descriptor(No.1):0x04***********************/
	//Operational Alternate Setting 1
	0x09,    //Length
	USB_DESC_TYPE_INTERFACE,    //DescriptorType:Interface
	AUDIO_IN_IF,    //InterfaceNum:1
	0x01,    //AlternateSetting:1
	0x01,    //NumEndpoint:1
	USB_DEVICE_CLASS_AUDIO,    //InterfaceClass:audio
	AUDIO_SUBCLASS_AUDIOSTREAMING,    //InterfaceSubClass:audio streaming
	AUDIO_PROTOCOL_UNDEFINED,    //InterfaceProtocol
	0x00,    //Interface String

	/*******************AS descriptor subtype Descriptor:0x24 0x01*************************/
	AUDIO_STREAMING_INTERFACE_DESC_SIZE,    //Length
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,    //DescriptorType:audio interface descriptor
	AUDIO_STREAMING_GENERAL,    //DescriptorSubType:AS_GENERAL
	0x03,    //TerminalLink:#3USB USB Streaming OT   //Linked to USB Streaming In Terminal
	0x00,    //Delay:0 接口延时
	0x01,
	0x00,    //FormatTag:PCM

   /****************** Audio Class Specific type I format INTERFACE Descriptor: 0x24 0X02 ***********/
   //设置音频流的格式
	0x0B,    //Length
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,    //DescriptorType:audio interface descriptor
	AUDIO_STREAMING_FORMAT_TYPE,    //DescriptorSubType:Format_type
	AUDIO_FORMAT_TYPE_I,    //FormatType:Format type 1
	0x01,    //NumberOfChanne:1
	0x02,    //SubframeSize:2byte
	16,    //BitsResolution:16bit
	0x01,    //SampleFreqType:One sampling frequency.
	AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ),     //采样率两个字节

	/******************************* Audio Recorder IN ENDPOINT descriptor: 0x05 *******************************/
	AUDIO_STANDARD_ENDPOINT_DESC_SIZE,    //Length
	USB_DESC_TYPE_ENDPOINT,    //DescriptorType:endpoint descriptor
	AUDIO_IN_EP,    //EndpointAddress:Input endpoint 2
	USBD_EP_TYPE_ISOC,
	MIC_PACKET_SZE(USBD_AUDIO_FREQ),
	0x01,    //Interval
	0x00,
	0x00,

	/******************************* Audio Class Specific ENDPOINT Descriptor: 0x25 0x01*******************************/
	AUDIO_STREAMING_ENDPOINT_DESC_SIZE,    //Length
	AUDIO_ENDPOINT_DESCRIPTOR_TYPE,    //DescriptorType:audio endpoint descriptor
	AUDIO_ENDPOINT_GENERAL,    //DescriptorSubType:audio endpiont general
	0x00,    //Attributes:0x00........
	0x00,    //LockDelayUnits
	0x00,
	0x00,    //LockDelay

	/***********************Audio Speaker Interface descriptor(No.2):0x04*****************************/
	//播放部分开始
	0x09,    //Length
	USB_DESC_TYPE_INTERFACE,    //DescriptorType:Interface
	AUDIO_OUT_IF,    //InterfaceNum:2
	0x00,    //AlternateSetting:0
	0x00,    //NumEndpoint:0
	USB_DEVICE_CLASS_AUDIO,    //InterfaceClass:audio
	AUDIO_SUBCLASS_AUDIOSTREAMING,    //InterfaceSubClass:audio streaming
	AUDIO_PROTOCOL_UNDEFINED,    //InterfaceProtocol
	0x00,    //Interface String

	/***********************Audio Speaker Interface descriptor(No.2):0x04*****************************/
	0x09,    //Length
	USB_DESC_TYPE_INTERFACE,    //DescriptorType:Interface
	AUDIO_OUT_IF,    //InterfaceNum:2
	0x01,    //AlternateSetting:1
#ifdef USE_SYNC_EP
	0x02,    //NumEndpoint:2            //这里包括一个反馈端点
#else
	0x01,
#endif
	USB_DEVICE_CLASS_AUDIO,    //InterfaceClass:audio
	AUDIO_SUBCLASS_AUDIOSTREAMING,    //InterfaceSubClass:audio streaming
	AUDIO_PROTOCOL_UNDEFINED,    //InterfaceProtocol
	0x00,    //Interface String

	/*******************AS_GENERAL descriptor subtype Descriptor:0x24 0x01*************************/
	AUDIO_STREAMING_INTERFACE_DESC_SIZE,    //Length
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,    //DescriptorType:audio interface descriptor
	AUDIO_STREAMING_GENERAL,    //DescriptorSubType:AS_GENERAL
	0x04,    //TerminalLink:#4 USB Streaming IT
	0x01,    //Delay:1
	0x01,
	0x00,    //FormatTag:PCM

	/****************** Audio Class Specific type I format INTERFACE Descriptor: 0x24 0X02 ***********/
	0x0B,    //Length
	AUDIO_INTERFACE_DESCRIPTOR_TYPE,    //DescriptorType:audio interface descriptor
	AUDIO_STREAMING_FORMAT_TYPE,    //DescriptorSubType:Format_type
	AUDIO_FORMAT_TYPE_I,    //FormatType:Format type 1
	0x02,    //NumberOfChanne:1
	0x02,    //SubframeSize: 2byte
	16,    //BitsResolution: 16bit
	0x01,    //SampleFreqType:One sampling frequency.
	AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ),

	/******************************* Audio Speaker OUT ENDPOINT descriptor: 0x05 *******************************/
	//播放端点描述符
	0x09,    //Length
	USB_DESC_TYPE_ENDPOINT,    //DescriptorType:endpoint descriptor
	AUDIO_OUT_EP,    //EndpointAddress:Output endpoint 01
#ifdef USE_SYNC_EP
	USBD_EP_TYPE_ISOC|0x04,    //Attributes:0x05,Isochronous,Synchronization Type(Asynchronous).........
#else
	USBD_EP_TYPE_ISOC,
#endif
	AUDIO_PACKET_SZE_16B(USBD_AUDIO_FREQ),
	0x01,    //Interval       AUDIO_OUT_PACKET
	0x00,           //没有使用
#ifdef USE_SYNC_EP
	SYNC_IN_EP,           //这个值是反馈端点的端点号 bSynchAddress：同步端点的地址
#else
	0x00,
#endif

	/******************************* Audio Class Specific ENDPOINT Descriptor: 0x25 0x01*******************************/
	AUDIO_STREAMING_ENDPOINT_DESC_SIZE,    //Length
	AUDIO_ENDPOINT_DESCRIPTOR_TYPE,    //DescriptorType:audio endpoint descriptor
	AUDIO_ENDPOINT_GENERAL,    //DescriptorSubType:audio endpiont general
	0x01,    //Attributes:0x00.............
	0x00,    //LockDelayUnits
	0x00,
	0x00,    //LockDelay
	/* Endpoint 2 - Standard Descriptor - See UAC Spec 1.0 p.63 4.6.2.1 Standard AS Isochronous Synch Endpoint Descriptor */
#ifdef USE_SYNC_EP
	AUDIO_STANDARD_ENDPOINT_DESC_SIZE, /* bLength */
	USB_DESC_TYPE_ENDPOINT,            /* bDescriptorType */
	SYNC_IN_EP,                       /* bEndpointAddress */
	0x11,                 			  /* bmAttributes */
	0x03,
	0x00,                        /* wMaxPacketSize in Bytes */
	0x01,                              /* bInterval 1ms */
	SOF_RATE,                          /* bRefresh 4ms = 2^2 */
	0x00,                              /* bSynchAddress */
#endif

	 /************** Descriptor of CUSTOM HID interface ****************/
	  /* 09 */
	0x09,         /*bLength: Interface Descriptor size*/
	USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
	HID_CTRL_IF,  /*bInterfaceNumber: Number of Interface*/
	0x00,         /*bAlternateSetting: Alternate setting*/
	0x02,         /*bNumEndpoints*/
	0x03,         /*bInterfaceClass: CUSTOM_HID*/
	0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
	0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
	0,            /*iInterface: Index of string descriptor*/
	/******************** Descriptor of CUSTOM_HID *************************/
	/* 18 */
	0x09,         /*bLength: CUSTOM_HID Descriptor size*/
	CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
	0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
	0x01,
	0x00,         /*bCountryCode: Hardware target country*/
	0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
	0x22,         /*bDescriptorType*/
	USBD_CUSTOM_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
	0x00,
	/******************** Descriptor of Custom HID endpoints ********************/
	/* 27 */
	0x07,          /*bLength: Endpoint Descriptor size*/
	USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
	CUSTOM_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
	0x03,          /*bmAttributes: Interrupt endpoint*/
	CUSTOM_HID_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
	0x00,
	CUSTOM_HID_FS_BINTERVAL,          /*bInterval: Polling Interval */
	  /* 34 */

	0x07,          /* bLength: Endpoint Descriptor size */
	USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
	CUSTOM_HID_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
	0x03, /* bmAttributes: Interrupt endpoint */
	CUSTOM_HID_EPOUT_SIZE,  /* wMaxPacketSize: 2 Bytes max  */
	0x00,
	CUSTOM_HID_FS_BINTERVAL,  /* bInterval: Polling Interval */
	  /* 41 */
   /*---------------------------------------------------------------------------*/
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

void USBD_COMP_AUDIO_UpdateOutBuffer(void)
{
	switchToClass (&hUsbDeviceFS, &comp_dev[UAC]);
	USBD_AUDIO_UpdateOutBuffer(&hUsbDeviceFS);
}

void USBD_COMP_AUDIO_UpdateInBuffer(void)
{
	switchToClass (&hUsbDeviceFS, &comp_dev[UAC]);
	USBD_AUDIO_UpdateInBuffer(&hUsbDeviceFS);
}

/****END OF FILE****/
