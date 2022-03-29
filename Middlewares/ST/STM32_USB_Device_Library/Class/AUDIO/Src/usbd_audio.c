/**
  ******************************************************************************
  * @file    usbd_audio.c
  * @author  MCD Application Team
  * @brief   This file provides the Audio core functions.
  *
  * @verbatim
  *
  *          ===================================================================
  *                                AUDIO Class  Description
  *          ===================================================================
 *           This driver manages the Audio Class 1.0 following the "USB Device Class Definition for
  *           Audio Devices V1.0 Mar 18, 98".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Standard AC Interface Descriptor management
  *             - 1 Audio Streaming Interface (with single channel, PCM, Stereo mode)
  *             - 1 Audio Streaming Endpoint
  *             - 1 Audio Terminal Input (1 channel)
  *             - Audio Class-Specific AC Interfaces
  *             - Audio Class-Specific AS Interfaces
  *             - AudioControl Requests: only SET_CUR and GET_CUR requests are supported (for Mute)
  *             - Audio Feature Unit (limited to Mute control)
  *             - Audio Synchronization type: Asynchronous
  *             - Single fixed audio sampling rate (configurable in usbd_conf.h file)
  *          The current audio class version supports the following audio features:
  *             - Pulse Coded Modulation (PCM) format
  *             - sampling rate: 48KHz.
  *             - Bit resolution: 16
  *             - Number of channels: 2
  *             - No volume control
  *             - Mute/Unmute capability
  *             - Asynchronous Endpoints
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      http://www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* BSPDependencies
  - "stm32xxxxx_{eval}{discovery}.c"
  - "stm32xxxxx_{eval}{discovery}_io.c"
  - "stm32xxxxx_{eval}{discovery}_audio.c"
  EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio.h"
#include "usbd_ctlreq.h"

#include "tlv320aic3204.h"
/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_AUDIO
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_AUDIO_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Defines
  * @{
  */
/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_FunctionPrototypes
  * @{
  */

static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef* pdev, uint8_t cfgidx);

static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef* pdev, uint8_t cfgidx);

static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);

static uint8_t* USBD_AUDIO_GetCfgDesc(uint16_t* length);

static uint8_t* USBD_AUDIO_GetDeviceQualifierDesc(uint16_t* length);

static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef* pdev, uint8_t epnum);

static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef* pdev, uint8_t epnum);

static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef* pdev);

static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef* pdev);

static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef* pdev);

static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef* pdev, uint8_t epnum);

static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef* pdev, uint8_t epnum);

static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);
static void AUDIO_REQ_GetMax(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);
static void AUDIO_REQ_GetMin(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);
static void AUDIO_REQ_GetRes(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);

static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req);

static void AUDIO_OUT_StopAndReset(USBD_HandleTypeDef* pdev);

static void AUDIO_OUT_Restart(USBD_HandleTypeDef* pdev);

static void AUDIO_IN_StopAndReset(USBD_HandleTypeDef* pdev);

static void AUDIO_IN_Restart(USBD_HandleTypeDef* pdev);

static void Prepare_IN_Packet(USBD_HandleTypeDef* pdev);

static int8_t VOL_PERCENT(int16_t vol);

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Variables
  * @{
  */

USBD_ClassTypeDef USBD_AUDIO = {
    USBD_AUDIO_Init,
    USBD_AUDIO_DeInit,
    USBD_AUDIO_Setup,
    USBD_AUDIO_EP0_TxReady,
    USBD_AUDIO_EP0_RxReady,
    USBD_AUDIO_DataIn,
    USBD_AUDIO_DataOut,
    USBD_AUDIO_SOF,
    USBD_AUDIO_IsoINIncomplete,
    USBD_AUDIO_IsoOutIncomplete,
    USBD_AUDIO_GetCfgDesc,
    USBD_AUDIO_GetCfgDesc,
    USBD_AUDIO_GetCfgDesc,
    USBD_AUDIO_GetDeviceQualifierDesc,
};

/* USB AUDIO device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_CfgDesc[USB_AUDIO_CONFIG_DESC_SIZ] __ALIGN_END =
{
    /* Configuration 1 */
    0x09,                                 /* bLength */
    USB_DESC_TYPE_CONFIGURATION,          /* bDescriptorType */
    LOBYTE(USB_AUDIO_CONFIG_DESC_SIZ),    /* wTotalLength  109 bytes*/
    HIBYTE(USB_AUDIO_CONFIG_DESC_SIZ),
    0x03,    //NumInterfaces:3
    0x01,    //ConfigurationValue
    0x00,    //Configuration String
    0x80,    //Attributes:Bus Power
    0xFA,    //MaxPower = 0xfa*2ma

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

    /******************* Audio Class Specific type of Input Terminal:0x24 0x02*************************/
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
    SYNC_IN_EP,                        /* bEndpointAddress */
    0x11,                 			   /* bmAttributes */
    0x03,
	0x00,                        	   /* wMaxPacketSize in Bytes */
    0x01,                              /* bInterval 1ms */
    SOF_RATE,                          /* bRefresh 4ms = 2^2 */
    0x00,                              /* bSynchAddress */
#endif
} ;

/**
 * USB Standard Device Descriptor
 * @see https://www.keil.com/pack/doc/mw/USB/html/_u_s_b__device__qualifier__descriptor.html
 */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END = {
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

volatile uint32_t tx_flag = 1;
volatile uint32_t is_out_ready = 0;
volatile uint8_t isDataReceivedFromUSB = 0;

#ifdef USE_SYNC_EP
	volatile int32_t gap,corr;
	volatile int32_t shift = 0;
	static volatile  uint16_t SOF_num=0;
	uint32_t feedback_data __attribute__ ((aligned(4))) = 0x0C0000;
	uint32_t accum __attribute__ ((aligned(4))) = 0x0C0000;
	static volatile uint8_t dpid;
	volatile uint32_t sync_in_fnsof = 0;
#endif

volatile uint32_t audio_in_fnsof = 0;

uint16_t outDataBuffer[AUDIO_TOTAL_OUT_BUF_SIZE] = {0};
uint16_t inDataBuffer[AUDIO_TOTAL_IN_BUF_SIZE] = {0};
uint8_t outPacketBuffer[AUDIO_OUT_PACKET+16U] = {0};
uint8_t inPacketBuffer[AUDIO_IN_PACKET+2U] = {0};

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Functions
  * @{
  */

/**
  * @brief  USBD_AUDIO_Init
  *         Initialize the AUDIO interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef* pdev, uint8_t cfgidx)
{
  USBD_AUDIO_HandleTypeDef* haudio;

  /* Open EP OUT */
  USBD_LL_OpenEP(pdev, AUDIO_OUT_EP, USBD_EP_TYPE_ISOC, AUDIO_OUT_PACKET+16U);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 1U;
	
#ifdef USE_SYNC_EP
	/* Open EP IN (sync)*/
  USBD_LL_OpenEP(pdev, SYNC_IN_EP, USBD_EP_TYPE_ISOC, SYNC_IN_PACKET);
  pdev->ep_in[SYNC_IN_EP & 0xFU].is_used = 1U;
  USBD_LL_FlushEP(pdev, SYNC_IN_EP);
#endif
	
	/* Open EP IN (mic) */
  USBD_LL_OpenEP (pdev, AUDIO_IN_EP, USBD_EP_TYPE_ISOC, AUDIO_IN_PACKET+2U);
  pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 1U;

  /* Flush IN endpoints */
  USBD_LL_FlushEP(pdev, AUDIO_IN_EP);

  /**
   * Set tx_flag 1 to block feedback transmission in SOF handler since
   * device is not ready.
   */
  tx_flag = 1U;
  is_out_ready = 0;

  /* Allocate Audio structure */
  pdev->pClassData = (void*)USBD_malloc(sizeof(USBD_AUDIO_HandleTypeDef));

  if (pdev->pClassData == NULL) {
    return USBD_FAIL;
  } else {
    haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;
    for (uint8_t i = 0; i <= USBD_MAX_NUM_INTERFACES; i++)
    {
      haudio->alt_setting[i] = 0U;
    }
    haudio->out_offset = AUDIO_OFFSET_UNKNOWN;
    haudio->out_wr_ptr = 0U;
    haudio->out_rd_ptr = 0U;
    haudio->out_rd_enable = 0U;
    haudio->vol = USBD_AUDIO_VOL_DEFAULT;

    haudio->in_packet_buffer_enable = 0U;
    haudio->in_offset = AUDIO_OFFSET_UNKNOWN;
    haudio->in_rd_ptr = 0U;
    haudio->in_wr_ptr = 0U;
    haudio->in_packet_size = AUDIO_IN_PACKET;

	haudio->in_buffer = inDataBuffer;
	haudio->in_packet_buffer = inPacketBuffer;
	haudio->out_buffer = outDataBuffer;
	haudio->out_packet_buffer = outPacketBuffer;

	USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, haudio->out_packet_buffer, AUDIO_OUT_PACKET+16U);

    /* Initialize the Audio output Hardware layer */
    if (((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->Init(USBD_AUDIO_FREQ, VOL_PERCENT(haudio->vol), 0U) != 0) {
      return USBD_FAIL;
    }
  }
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Init
  *         DeInitialize the AUDIO layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef* pdev,
                                 uint8_t cfgidx)
{
  /* Flush all endpoints */
  USBD_LL_FlushEP(pdev, AUDIO_OUT_EP);
  USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
	
#ifdef USE_SYNC_EP
	USBD_LL_FlushEP(pdev, SYNC_IN_EP);
	/* Close EP IN (sync)*/
  USBD_LL_CloseEP(pdev, SYNC_IN_EP);
  pdev->ep_in[SYNC_IN_EP & 0xFU].is_used = 0U;
#endif

  /* Close EP OUT */
  USBD_LL_CloseEP(pdev, AUDIO_OUT_EP);
  pdev->ep_out[AUDIO_OUT_EP & 0xFU].is_used = 0U;
	
  /* Close EP IN (mic) */
  USBD_LL_CloseEP(pdev, AUDIO_IN_EP);
  pdev->ep_in[AUDIO_IN_EP & 0xFU].is_used = 0U;

  /* Clear feedback transmission flag */
  tx_flag = 0U;

  /* DeInit physical Interface components */
  if (pdev->pClassData != NULL) {
    ((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->DeInit(0U);
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Setup
  *         Handle the AUDIO specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef* pdev,
                                USBD_SetupReqTypedef* req)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  uint16_t len;
  uint8_t* pbuf;
  uint16_t status_info = 0U;
  uint8_t ret = USBD_OK;

  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  switch (req->bmRequest & USB_REQ_TYPE_MASK) {
    /* AUDIO Class Requests */
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest) {
        case AUDIO_REQ_GET_CUR:
          AUDIO_REQ_GetCurrent(pdev, req);
          break;

        case AUDIO_REQ_GET_MAX:
          AUDIO_REQ_GetMax(pdev, req);
          break;

        case AUDIO_REQ_GET_MIN:
          AUDIO_REQ_GetMin(pdev, req);
          break;

        case AUDIO_REQ_GET_RES:
          AUDIO_REQ_GetRes(pdev, req);
          break;

        case AUDIO_REQ_SET_CUR:
          AUDIO_REQ_SetCurrent(pdev, req);
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    /* Standard Requests */
    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest) {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            USBD_CtlSendData(pdev, (uint8_t*)(void*)&status_info, 2U);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
          if ((req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE) {
            pbuf = USBD_AUDIO_CfgDesc + 18;
            len = MIN(USB_AUDIO_DESC_SIZ, req->wLength);

            USBD_CtlSendData(pdev, pbuf, len);
          }
          break;

        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            USBD_CtlSendData(pdev, (uint8_t*)&haudio->alt_setting[req->wIndex], 1U);
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED) {
            if ((uint8_t)(req->wValue) <= USBD_MAX_NUM_INTERFACES) {
                /* Handles Alternate Settings 0 of Audio OUT interface */
            	if(req->wIndex == AUDIO_OUT_IF)
            	{
                    /* Do things only when alt_setting changes */
            		if(haudio->alt_setting[AUDIO_OUT_IF] != (uint8_t)(req->wValue))
            		{
            			haudio->alt_setting[AUDIO_OUT_IF] = (uint8_t) (req->wValue);
						if (haudio->alt_setting[AUDIO_OUT_IF] == 1U)
						{
							is_out_ready = 1U;
							AUDIO_OUT_Restart(pdev);
						}
						else
						{
							is_out_ready = 0U;
							tx_flag = 0U;
							AUDIO_OUT_StopAndReset(pdev);
						#ifdef USE_SYNC_EP
							USBD_LL_FlushEP(pdev, SYNC_IN_EP);
						#endif
							USBD_LL_FlushEP(pdev, AUDIO_OUT_EP);
						}
            		}
            	}

                /* Handles Alternate Settings 1 of Audio IN interface */
            	if(req->wIndex == AUDIO_IN_IF)
            	{
                    /* Do things only when alt_setting changes */
            		if(haudio->alt_setting[AUDIO_IN_IF] != (uint8_t)(req->wValue))
            		{
            			haudio->alt_setting[AUDIO_IN_IF] = (uint8_t) (req->wValue);
						if (haudio->alt_setting[AUDIO_IN_IF] == 1U)
						{
							if (haudio->in_packet_buffer_enable == 0U)
							{
							  /* Open EP IN (mic) */
								AUDIO_IN_Restart(pdev);
							}
						}
						else
						{
							AUDIO_IN_StopAndReset(pdev);
						}
            		}
            	}
            } else {
              /* Call the error management function (command will be nacked */
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
          } else {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;
    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return ret;
}

/**
  * @brief  USBD_AUDIO_GetCfgDesc
  *         return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t* USBD_AUDIO_GetCfgDesc(uint16_t* length)
{
  *length = sizeof(USBD_AUDIO_CfgDesc);
  return USBD_AUDIO_CfgDesc;
}

/**
  * @brief  USBD_AUDIO_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef* pdev,
                                 uint8_t epnum)
{
  USBD_AUDIO_HandleTypeDef *haudio;

  haudio = (USBD_AUDIO_HandleTypeDef*) pdev->pClassData;
  if (haudio == NULL)
  {
		return (uint8_t)USBD_FAIL;
  }
#ifdef USE_SYNC_EP
  if (epnum == (SYNC_IN_EP & 0x7F))
  {
    tx_flag = 0U;
    SOF_num = 0;
  }
#endif
  // send mic data TODO: fix sync fault when audio in data transmit
  if (epnum == (AUDIO_IN_EP & 0x7F))
  {
	  Prepare_IN_Packet(pdev);

	  audio_in_fnsof = USB_SOF_NUMBER();

	  USBD_LL_FlushEP (pdev, AUDIO_IN_EP);
	  USBD_LL_Transmit (pdev, AUDIO_IN_EP, haudio->in_packet_buffer, haudio->in_packet_size);
  }
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef* pdev)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;
  /**
   * 1. Must be static so that the values are kept when the function is
   *    again called.
   * 2. Must be volatile so that it will not be optimized out by the compiler.
   */
  static volatile uint32_t sof_count_in = 0;
  static volatile uint16_t in_gap = 0;
  uint16_t out_rd_ptr_offset = 0;
  uint16_t in_rd_ptr_offset = 0;

#ifndef USE_SYNC_EP
  static volatile uint32_t sof_count_out = 0;
  static volatile uint16_t out_gap = 0;
#endif

  /* Do stuff only when playing */
  if (haudio->out_rd_enable == 1U && is_out_ready == 1U) {
	// wait for stabilization of data pointers stabilization before calculating feedback data
	/* Remaining writable buffer size */
	uint32_t audio_out_buf_writable_size;

	/* Update audio out read pointer */
	out_rd_ptr_offset = AUDIO_TOTAL_OUT_BUF_SIZE/4 - tlv320aic3204_drv->GetOutDataRemainingSize();

	/* Calculate remaining writable buffer size */
	if ((haudio->out_rd_ptr + out_rd_ptr_offset) < haudio->out_wr_ptr) {
	  audio_out_buf_writable_size = haudio->out_rd_ptr + out_rd_ptr_offset + AUDIO_TOTAL_OUT_BUF_SIZE - haudio->out_wr_ptr;
	} else {
	  audio_out_buf_writable_size = haudio->out_rd_ptr + out_rd_ptr_offset - haudio->out_wr_ptr;
	}

#ifdef USE_SYNC_EP
	/* Monitor remaining writable buffer size with LED */
	if (audio_out_buf_writable_size < AUDIO_BUF_SAFEZONE) {
		LED_GPIO_Port->ODR |= LED_Pin;
	} else {
		LED_GPIO_Port->ODR &= ~LED_Pin;
	}

	shift = 0;
	gap = (haudio->out_wr_ptr - haudio->out_rd_ptr - out_rd_ptr_offset);

	if (gap < 0)
	{
		gap += (AUDIO_TOTAL_OUT_BUF_SIZE);
	}
	shift = ((AUDIO_TOTAL_OUT_BUF_SIZE/2) - gap)>>3;
	accum += (TIM2->CCR1);

	if (shift != 0)
	{
		accum += shift;
	}
	SOF_num++;
	if (SOF_num==(1<<SOF_RATE))
	{
		feedback_data += (accum<<(6-SOF_RATE));
		feedback_data >>= 1;
//		 limit feedback data values in 47 - 49 kHz
		if(feedback_data < 0x0BC000) feedback_data = 0x0BC000;
		if(feedback_data > 0x0C4000) feedback_data = 0x0C4000;

		SOF_num=0;
		accum=0;
	}
#else
	if (sof_count_out++ == (1<<SOF_RATE)) {
		sof_count_out = 0;
	  /* Calculate feedback value based on the change of writable buffer size */
		if(audio_out_buf_writable_size >= (3*(AUDIO_TOTAL_OUT_BUF_SIZE>>3) + out_gap) && audio_out_buf_writable_size <= (5*(AUDIO_TOTAL_OUT_BUF_SIZE>>3) - out_gap))
		{
			tlv320aic3204_drv->SetFrequencyDeviation(0);
			LED_GPIO_Port->ODR &= ~LED_Pin;
			out_gap = 0;
		}
		else if(audio_out_buf_writable_size < 3*(AUDIO_TOTAL_OUT_BUF_SIZE>>3) + out_gap)
		{
			tlv320aic3204_drv->SetFrequencyDeviation(2);
			LED_GPIO_Port->ODR ^= LED_Pin;
			out_gap = AUDIO_TOTAL_OUT_BUF_SIZE>>4;
		}
		else if(audio_out_buf_writable_size > 5*(AUDIO_TOTAL_OUT_BUF_SIZE>>3) - out_gap)
		{
			tlv320aic3204_drv->SetFrequencyDeviation(1);
			LED_GPIO_Port->ODR |= LED_Pin;
			out_gap = AUDIO_TOTAL_OUT_BUF_SIZE>>4;
		}

	}
#endif

#ifdef USE_SYNC_EP
	/* Transmit feedback only when the last one is transmitted */
	if (tx_flag == 0U) {
		/* Get FNSOF. Use volatile for fnsof_new since its address is mapped to a hardware register. */
		uint32_t volatile fnsof_new = USB_SOF_NUMBER();

		if ((fnsof_new & 0x1) == dpid)
		{
			sync_in_fnsof = fnsof_new;

			USBD_LL_Transmit(pdev, SYNC_IN_EP, (uint8_t*)&feedback_data, 3);
			/* Block transmission until it's finished. */
			tx_flag = 1U;
		}
	}
#endif
  }

  // update audio in read pointer
  if (haudio->in_packet_buffer_enable > 0U)
  {
	  /* Remaining writable buffer size */
	  uint32_t audio_buf_writable_size;
	  in_rd_ptr_offset = AUDIO_TOTAL_IN_BUF_SIZE/4 - tlv320aic3204_drv->GetInDataRemainingSize();

	  /* Calculate remaining writable buffer size */
	  if ((haudio->in_rd_ptr + in_rd_ptr_offset) < haudio->in_wr_ptr) {
	    audio_buf_writable_size = haudio->in_rd_ptr + in_rd_ptr_offset + AUDIO_TOTAL_IN_BUF_SIZE - haudio->in_wr_ptr;
	  } else {
	    audio_buf_writable_size = haudio->in_rd_ptr + in_rd_ptr_offset - haudio->in_wr_ptr;
	  }

	  if (sof_count_in++ == (1<<SOF_RATE)) {
	    sof_count_in = 0;
	    /* Set in packet size value based on the change of writable buffer size */
	    if(audio_buf_writable_size >= (3*(AUDIO_TOTAL_IN_BUF_SIZE>>3) + in_gap) && audio_buf_writable_size <= (5*(AUDIO_TOTAL_IN_BUF_SIZE>>3) - in_gap))
	    {
	    	haudio->in_packet_size = AUDIO_IN_PACKET;
	    	LED_GPIO_Port->ODR &= ~LED_Pin;
	    	in_gap = 0;
	    }
	    else if(audio_buf_writable_size < 3*(AUDIO_TOTAL_IN_BUF_SIZE>>3) + in_gap)
	    {
	    	haudio->in_packet_size = AUDIO_IN_PACKET-2U;
	    	LED_GPIO_Port->ODR ^= LED_Pin;
	    	in_gap = AUDIO_TOTAL_IN_BUF_SIZE>>4;
	    }
	    else if(audio_buf_writable_size > 5*(AUDIO_TOTAL_IN_BUF_SIZE>>3) - in_gap)
	    {
	    	haudio->in_packet_size = AUDIO_IN_PACKET+2U;
	    	LED_GPIO_Port->ODR |= LED_Pin;
	    	in_gap = AUDIO_TOTAL_IN_BUF_SIZE>>4;
	    }
	  }
  }

  /* Prepare AUDIO IN endpoint to send 1st packet */
  if (haudio->in_packet_buffer_enable == 2U && haudio->in_rd_ptr >= AUDIO_TOTAL_IN_BUF_SIZE / 2U)
  {
	  haudio->in_packet_buffer_enable = 1U;

	  Prepare_IN_Packet(pdev);
	  audio_in_fnsof = USB_SOF_NUMBER();

	  USBD_LL_FlushEP (pdev, AUDIO_IN_EP);
	  USBD_LL_Transmit (pdev, AUDIO_IN_EP, haudio->in_packet_buffer, haudio->in_packet_size);
  }

  return USBD_OK;
}

/**
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * USBD_AUDIO_IsoINIncomplete & USBD_AUDIO_IsoOutIncomplete are not
 * enabled by default.
 *
 * Go to Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
 * Fill in USBD_LL_IsoINIncomplete and USBD_LL_IsoOUTIncomplete with
 * actual handler functions.
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 */

/**
  * @brief  USBD_AUDIO_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef* pdev, uint8_t epnum)
{
	uint16_t current_sof;
	current_sof = USB_SOF_NUMBER();
	// check incomplete transfer for AUDIO_IN_EP
	if(IS_ISO_IN_INCOMPLETE_EP((AUDIO_IN_EP & 0x7F), current_sof, audio_in_fnsof))
	{
		USB_CLEAR_INCOMPLETE_IN_EP(AUDIO_IN_EP);
		USBD_LL_FlushEP(pdev, AUDIO_IN_EP);
	}

#ifdef USE_SYNC_EP
	// check incomplete transfer for SYNC_IN_EP
	if(IS_ISO_IN_INCOMPLETE_EP((SYNC_IN_EP & 0x7F), current_sof, sync_in_fnsof))
	{
		USB_CLEAR_INCOMPLETE_IN_EP(SYNC_IN_EP);

		dpid = current_sof & 0x01;

		if (tx_flag == 1U)
		{
			USBD_LL_FlushEP(pdev, SYNC_IN_EP);
			tx_flag = 0U;
		}
	}
#endif
  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef* pdev, uint8_t epnum)
{
	return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef* pdev,
                                  uint8_t epnum)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  if (epnum == AUDIO_OUT_EP && is_out_ready) {
    uint32_t curr_length = USBD_GetRxCount(pdev, epnum);
    /* Ignore strangely large packets */
	uint32_t num_of_samples = 0U;
	uint32_t i, packet_data_ptr = 0U;

	num_of_samples = curr_length / 4;

	for (i = 0; i < num_of_samples; i++) {
		/* Copy one sample */
		haudio->out_buffer[haudio->out_wr_ptr] = (uint16_t)((haudio->out_packet_buffer[packet_data_ptr + 1] << 8)|(haudio->out_packet_buffer[packet_data_ptr]));
		/* Rollback if reach end of buffer */
		haudio->out_wr_ptr++;
		if (haudio->out_wr_ptr == AUDIO_TOTAL_OUT_BUF_SIZE) {
			haudio->out_wr_ptr = 0U;
		}

		haudio->out_buffer[haudio->out_wr_ptr] = (uint16_t)((haudio->out_packet_buffer[packet_data_ptr + 3] << 8)|(haudio->out_packet_buffer[packet_data_ptr + 2]));
		/* Rollback if reach end of buffer */
		haudio->out_wr_ptr++;
		if (haudio->out_wr_ptr == AUDIO_TOTAL_OUT_BUF_SIZE) {
			haudio->out_wr_ptr = 0U;
		}

		packet_data_ptr += 4U;
	}

	/* Start playing when half of the audio buffer is filled */
	if (haudio->out_offset == AUDIO_OFFSET_UNKNOWN) {
		if (haudio->out_wr_ptr >= AUDIO_TOTAL_OUT_BUF_SIZE / 2U)
		{
			haudio->out_offset = AUDIO_OFFSET_NONE;

			if (haudio->out_rd_enable == 0U) {
				haudio->out_rd_enable = 1U;
			}
			((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->AudioCmd(haudio->out_buffer, AUDIO_TOTAL_OUT_BUF_SIZE/4, AUDIO_CMD_START);
		}
	}
	// set data received flag for starting DMA transmit
	isDataReceivedFromUSB = 1U;
	USBD_LL_PrepareReceive(pdev, AUDIO_OUT_EP, haudio->out_packet_buffer, AUDIO_OUT_PACKET+16U);
  }

  return USBD_OK;
}

/**
 * @brief  AUDIO_Req_GetCurrent
 *         Handles the GET_CUR Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  if ((req->bmRequest & 0x1f) == AUDIO_CONTROL_REQ) {
    switch (HIBYTE(req->wValue)) {
      case AUDIO_CONTROL_REQ_FU_MUTE: {
        /* Current mute state */
        uint8_t mute = 0;
        USBD_CtlSendData(pdev, &mute, 1);
      };
          break;
      case AUDIO_CONTROL_REQ_FU_VOL: {
        /* Current volume. See UAC Spec 1.0 p.77 */
        USBD_CtlSendData(pdev, (uint8_t*)&haudio->vol, 2);
      };
          break;
    }
  } else if ((req->bmRequest & 0x1f) == AUDIO_STREAMING_REQ) {
    if (HIBYTE(req->wValue) == AUDIO_STREAMING_REQ_FREQ_CTRL) {
      /* Current frequency */
      uint32_t freq __attribute__((aligned(4))) = USBD_AUDIO_FREQ;
      USBD_CtlSendData(pdev, (uint8_t*)&freq, 3);
    }
  }
}

/**
 * @brief  AUDIO_Req_GetMax
 *         Handles the GET_MAX Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetMax(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
  if ((req->bmRequest & 0x1f) == AUDIO_CONTROL_REQ) {
    switch (HIBYTE(req->wValue)) {
      case AUDIO_CONTROL_REQ_FU_VOL: {
        int16_t vol_max = USBD_AUDIO_VOL_MAX;
        USBD_CtlSendData(pdev, (uint8_t*)&vol_max, 2);
      };
          break;
    }
  }
}

/**
 * @brief  AUDIO_Req_GetMin
 *         Handles the GET_MIN Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetMin(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
  if ((req->bmRequest & 0x1f) == AUDIO_CONTROL_REQ) {
    switch (HIBYTE(req->wValue)) {
      case AUDIO_CONTROL_REQ_FU_VOL: {
        int16_t vol_min = USBD_AUDIO_VOL_MIN;
        USBD_CtlSendData(pdev, (uint8_t*)&vol_min, 2);
      };
          break;
    }
  }
}

/**
 * @brief  AUDIO_Req_GetRes
 *         Handles the GET_RES Audio control request.
 * @param  pdev: instance
 * @param  req: setup class request
 * @retval status
 */
static void AUDIO_REQ_GetRes(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
  if ((req->bmRequest & 0x1f) == AUDIO_CONTROL_REQ) {
    switch (HIBYTE(req->wValue)) {
      case AUDIO_CONTROL_REQ_FU_VOL: {
        int16_t vol_res = USBD_AUDIO_VOL_STEP;
        USBD_CtlSendData(pdev, (uint8_t*)&vol_res, 2);
      };
          break;
    }
  }
}

/**
  * @brief  AUDIO_Req_SetCurrent
  *         Handles the SET_CUR Audio control request.
  * @param  pdev: instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef* pdev, USBD_SetupReqTypedef* req)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  if (req->wLength) {
    /* Prepare the reception of the buffer over EP0 */
    USBD_CtlPrepareRx(pdev,
                      haudio->control.data,
                      req->wLength);

    haudio->control.cmd = AUDIO_REQ_SET_CUR;          /* Set the request value */
    haudio->control.req_type = req->bmRequest & 0x1f; /* Set the request type. See UAC Spec 1.0 - 5.2.1 Request Layout */
    haudio->control.len = (uint8_t)req->wLength;      /* Set the request data length */
    haudio->control.unit = HIBYTE(req->wIndex);       /* Set the request target unit */
    haudio->control.cs = HIBYTE(req->wValue);         /* Set the request control selector (high byte) */
    haudio->control.cn = LOBYTE(req->wValue);         /* Set the request control number (low byte) */
  }
}

/**
  * @brief  USBD_AUDIO_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef* pdev)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  if (haudio->control.cmd == AUDIO_REQ_SET_CUR) { /* In this driver, to simplify code, only SET_CUR request is managed */

    if (haudio->control.req_type == AUDIO_CONTROL_REQ) {
      switch (haudio->control.cs) {
        /* Mute Control */
        case AUDIO_CONTROL_REQ_FU_MUTE: {
          ((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->MuteCtl(haudio->control.data[0]);
        };
            break;
        /* Volume Control */
        case AUDIO_CONTROL_REQ_FU_VOL: {
          int16_t vol = *(int16_t*)&haudio->control.data[0];
          haudio->vol = vol;
          ((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->VolumeCtl(VOL_PERCENT(vol));
        };
            break;
      }

    }

    haudio->control.req_type = 0U;
    haudio->control.cs = 0U;
    haudio->control.cn = 0U;
    haudio->control.cmd = 0U;
    haudio->control.len = 0U;
  }

  return USBD_OK;
}

/**
  * @brief  USBD_AUDIO_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef* pdev)
{
  /* Only OUT control data are processed */
  return USBD_OK;
}

/**
 * @brief  Stop playing and reset buffer pointers
 * @param  pdev: instance
 */
static void AUDIO_OUT_StopAndReset(USBD_HandleTypeDef* pdev)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

#ifndef USE_SYNC_EP
	tlv320aic3204_drv->SetFrequencyDeviation(0);
#endif

  haudio->out_offset = AUDIO_OFFSET_UNKNOWN;
  haudio->out_rd_enable = 0U;
  haudio->out_rd_ptr = 0U;
  haudio->out_wr_ptr = 0U;
}

/**
 * @brief  Restart playing with new parameters
 * @param  pdev: instance
 */
static void AUDIO_OUT_Restart(USBD_HandleTypeDef* pdev)
{
	USBD_LL_FlushEP(pdev, AUDIO_OUT_EP);
#ifdef USE_SYNC_EP
	USBD_LL_FlushEP(pdev, SYNC_IN_EP);

	tx_flag = 0U;
	SOF_num = 0;
#endif
}

static void AUDIO_IN_StopAndReset(USBD_HandleTypeDef* pdev)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  haudio->in_packet_buffer_enable = 0U;
  haudio->in_offset = AUDIO_OFFSET_UNKNOWN;
  haudio->in_rd_ptr = 0U;
  haudio->in_wr_ptr = 0U;

  USBD_LL_FlushEP(pdev, AUDIO_IN_EP);

//  ((USBD_AUDIO_ItfTypeDef*)pdev->pUserData)->AudioCmd(NULL, 0, AUDIO_CMD_STOP);
}

static void AUDIO_IN_Restart(USBD_HandleTypeDef* pdev)
{
  USBD_AUDIO_HandleTypeDef* haudio;
  haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

  haudio->in_packet_buffer_enable = 2U;
  haudio->in_packet_size = AUDIO_IN_PACKET;

  ((USBD_AUDIO_ItfTypeDef *) pdev->pUserData)->AudioCmd(haudio->in_buffer, AUDIO_TOTAL_IN_BUF_SIZE/4, AUDIO_CMD_RECORD);
}

static void Prepare_IN_Packet(USBD_HandleTypeDef* pdev)
{
	uint32_t num_of_samples;
	uint32_t i;
	uint16_t temp = 0;
	USBD_AUDIO_HandleTypeDef* haudio;
	haudio = (USBD_AUDIO_HandleTypeDef*)pdev->pClassData;

	num_of_samples = haudio->in_packet_size;

	for (i = 0; i < num_of_samples; i+=2) {
	  /* Copy one sample */
		temp = haudio->in_buffer[haudio->in_wr_ptr] - haudio->in_buffer[haudio->in_wr_ptr+1]; // calc difference between INxL and INxR samples
		haudio->in_wr_ptr += 2;
		haudio->in_packet_buffer[i] = (uint8_t)(temp & 0xFF);
		haudio->in_packet_buffer[i+1] = (uint8_t)((temp>>8) & 0xFF);

	  /* Rollback if reach end of buffer */
	  if (haudio->in_wr_ptr >= AUDIO_TOTAL_IN_BUF_SIZE) {
		haudio->in_wr_ptr = 0U;
	  }
	}
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
static uint8_t* USBD_AUDIO_GetDeviceQualifierDesc(uint16_t* length)
{
  *length = sizeof(USBD_AUDIO_DeviceQualifierDesc);
  return USBD_AUDIO_DeviceQualifierDesc;
}

/**
* @brief  USBD_AUDIO_RegisterInterface
* @param  fops: Audio interface callback
* @retval status
*/
uint8_t USBD_AUDIO_RegisterInterface(USBD_HandleTypeDef* pdev,
                                     USBD_AUDIO_ItfTypeDef* fops)
{
  if (fops != NULL) {
    pdev->pUserData = fops;
  }
  return USBD_OK;
}

/* Convert USB volume value to % */
int8_t VOL_PERCENT(int16_t vol)
{
	return (int8_t)(vol>>7);
}

void USBD_AUDIO_UpdateOutBuffer(USBD_HandleTypeDef* pdev)
{
	uint32_t BufferSize = AUDIO_TOTAL_OUT_BUF_SIZE;
	USBD_AUDIO_HandleTypeDef   *haudio;

	haudio = (USBD_AUDIO_HandleTypeDef *) pdev->pClassData;

	if(haudio->out_rd_enable == 1U)
	{
		// start DMA transmit only if data received from USB
		if(isDataReceivedFromUSB)
		{
			isDataReceivedFromUSB = 0U;
			haudio->out_rd_ptr += (uint16_t)BufferSize/4;

			if (haudio->out_rd_ptr == AUDIO_TOTAL_OUT_BUF_SIZE)
			{
			  /* roll back */
			  haudio->out_rd_ptr = 0U;
			}
			((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->AudioCmd(&haudio->out_buffer[haudio->out_rd_ptr],
																	 BufferSize/4,
																	 AUDIO_CMD_PLAY);
		}
		else
		{
			AUDIO_OUT_StopAndReset(pdev);
		}
	}
}

void USBD_AUDIO_UpdateInBuffer(USBD_HandleTypeDef* pdev)
{
	uint32_t BufferSize = AUDIO_TOTAL_IN_BUF_SIZE;
	USBD_AUDIO_HandleTypeDef   *haudio;
	haudio = (USBD_AUDIO_HandleTypeDef *) pdev->pClassData;

	if(haudio->in_packet_buffer_enable > 0)
	{
		haudio->in_rd_ptr += (uint16_t)BufferSize/4;

		if (haudio->in_rd_ptr == AUDIO_TOTAL_IN_BUF_SIZE)
		{
		  /* roll back */
		  haudio->in_rd_ptr = 0U;
		}
		((USBD_AUDIO_ItfTypeDef *)pdev->pUserData)->AudioCmd(&haudio->in_buffer[haudio->in_rd_ptr],
																 BufferSize/4,
																 AUDIO_CMD_RECORD);
	}
}

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