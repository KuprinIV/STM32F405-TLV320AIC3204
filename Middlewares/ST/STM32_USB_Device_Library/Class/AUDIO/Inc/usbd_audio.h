/**
  ******************************************************************************
  * @file    usbd_audio.h
  * @author  MCD Application Team
  * @brief   header file for the usbd_audio.c file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_AUDIO_H
#define __USB_AUDIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_AUDIO
  * @brief This file is the Header file for usbd_audio.c
  * @{
  */


/** @defgroup USBD_AUDIO_Exported_Defines
  * @{
  */

/**
 *
 * AUDIO Class Config
 *
 */

/* Frequency */
#ifndef USBD_AUDIO_FREQ
#define USBD_AUDIO_FREQ                      48000U
#endif

/* Volume. See UAC Spec 1.0 p.77 */
#ifndef USBD_AUDIO_VOL_DEFAULT
#define USBD_AUDIO_VOL_DEFAULT                        0x0000U
#endif

#ifndef USBD_AUDIO_VOL_MAX
#define USBD_AUDIO_VOL_MAX                            0x3000U
#endif

#ifndef USBD_AUDIO_VOL_MIN
#define USBD_AUDIO_VOL_MIN                            0x8001U
#endif

#ifndef USBD_AUDIO_VOL_STEP
#define USBD_AUDIO_VOL_STEP                           0x00AFU
#endif /* Total number of steps can't be too many, host will complain. */

/* Interface */
#ifndef USBD_MAX_NUM_INTERFACES
#define USBD_MAX_NUM_INTERFACES                       4U
#endif

/* bEndpointAddress, see UAC 1.0 spec, p.61 */
#define AUDIO_OUT_EP                                  0x03U
#define AUDIO_IN_EP                                   0x81U
#define SYNC_IN_EP                                    0x83U

#define AUDIO_CTRL_IF                                 0x00U
#define AUDIO_IN_IF                                   0x01U
#define AUDIO_OUT_IF                                  0x02U

#define SOF_RATE                                      0x05U

#define USB_AUDIO_CONFIG_DESC_SIZ                     201

#define AUDIO_INTERFACE_DESC_SIZE                     0x09U
#define USB_AUDIO_DESC_SIZ                            0x09U
#define AUDIO_STANDARD_ENDPOINT_DESC_SIZE             0x09U
#define AUDIO_STREAMING_ENDPOINT_DESC_SIZE            0x07U

#define AUDIO_DESCRIPTOR_TYPE                         0x21U
#define USB_DEVICE_CLASS_AUDIO                        0x01U
#define AUDIO_SUBCLASS_AUDIOCONTROL                   0x01U
#define AUDIO_SUBCLASS_AUDIOSTREAMING                 0x02U
#define AUDIO_PROTOCOL_UNDEFINED                      0x00U
#define AUDIO_STREAMING_GENERAL                       0x01U
#define AUDIO_STREAMING_FORMAT_TYPE                   0x02U

/* Audio Descriptor Types */
#define AUDIO_INTERFACE_DESCRIPTOR_TYPE               0x24U
#define AUDIO_ENDPOINT_DESCRIPTOR_TYPE                0x25U

/* Audio Control Interface Descriptor Subtypes */
#define AUDIO_CONTROL_HEADER                          0x01U
#define AUDIO_CONTROL_INPUT_TERMINAL                  0x02U
#define AUDIO_CONTROL_OUTPUT_TERMINAL                 0x03U
#define AUDIO_CONTROL_FEATURE_UNIT                    0x06U

#define AUDIO_INPUT_TERMINAL_DESC_SIZE                0x0CU
#define AUDIO_OUTPUT_TERMINAL_DESC_SIZE               0x09U
#define AUDIO_STREAMING_INTERFACE_DESC_SIZE           0x07U

#define AUDIO_CONTROL_MUTE                            0x0001U
#define AUDIO_CONTROL_VOL                             0x0002U

#define AUDIO_FORMAT_TYPE_I                           0x01U
#define AUDIO_FORMAT_TYPE_III                         0x03U

#define AUDIO_ENDPOINT_GENERAL                        0x01U

/* Audio Requests */
#define AUDIO_REQ_GET_CUR                             0x81U
#define AUDIO_REQ_GET_MIN                             0x82U
#define AUDIO_REQ_GET_MAX                             0x83U
#define AUDIO_REQ_GET_RES                             0x84U
#define AUDIO_REQ_SET_CUR                             0x01U
#define AUDIO_REQ_SET_MIN                             0x02U
#define AUDIO_REQ_SET_MAX                             0x03U
#define AUDIO_REQ_SET_RES                             0x04U

#define AUDIO_OUT_STREAMING_CTRL                      0x02U

/* Audio Control Requests */
#define AUDIO_CONTROL_REQ                             0x01U
/* Feature Unit, UAC Spec 1.0 p.102 */
#define AUDIO_CONTROL_REQ_FU_MUTE                     0x01U
#define AUDIO_CONTROL_REQ_FU_VOL                      0x02U

/* Audio Streaming Requests */
#define AUDIO_STREAMING_REQ                           0x02U
#define AUDIO_STREAMING_REQ_FREQ_CTRL                 0x01U
#define AUDIO_STREAMING_REQ_PITCH_CTRL                0x02U

/*
 * Max packet size: (freq / 1000 + extra_samples) * channels * bytes_per_sample
 * e.g. (96000 / 1000 + 1) * 2(stereo) * 2(16bit) = 388
 */
#define AUDIO_OUT_PACKET                          	((uint16_t)((USBD_AUDIO_FREQ / 1000U) * 2U * 2U))
#define AUDIO_IN_PACKET 							((uint16_t)((USBD_AUDIO_FREQ / 1000U) * 1U * 2U))
/* Input endpoint is for feedback. See USB 1.1 Spec, 5.10.4.2 Feedback. */
#define SYNC_IN_PACKET                               3U

/* Number of sub-packets in the audio transfer buffer. You can modify this value but always make sure
  that it is an even number and higher than 3 */
#define AUDIO_PACKET_NUM                          8U

/* Total size of the audio transfer buffer */
#define AUDIO_TOTAL_BUF_SIZE                          ((uint16_t)((USBD_AUDIO_FREQ / 1000U) * 2U * 4U * AUDIO_PACKET_NUM))

// USB macros
#define USB_DIEPCTL(ep_addr) ((USB_OTG_INEndpointTypeDef *)((uint32_t)USB_OTG_FS + USB_OTG_IN_ENDPOINT_BASE   \
        + (ep_addr&0x7FU)*USB_OTG_EP_REG_SIZE))->DIEPCTL
#define USB_DOEPCTL(ep_addr) ((USB_OTG_OUTEndpointTypeDef *)((uint32_t)USB_OTG_FS +  \
      USB_OTG_OUT_ENDPOINT_BASE + (ep_addr)*USB_OTG_EP_REG_SIZE))->DOEPCTL

#define USB_CLEAR_INCOMPLETE_IN_EP(ep_addr)     if((((ep_addr) & 0x80U) == 0x80U)){  \
            USB_DIEPCTL(ep_addr) |= (USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK);  \
                                         };
#define USB_RESTART_IN_EP(ep_addr)     if((((ep_addr) & 0x80U) == 0x80U)){  \
            USB_DIEPCTL(ep_addr) |= (USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK);  \
                                         };
#define USB_CLEAR_INCOMPLETE_OUT_EP(ep_addr)     if((((ep_addr) & 0x80U) == 0x00U)){  \
            USB_DOEPCTL(ep_addr) |= (USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK);  \
                                         };
#define USB_RESTART_OUT_EP(ep_addr)     if((((ep_addr) & 0x80U) == 0x00U)){  \
            USB_DOEPCTL(ep_addr) |= (USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK);  \
                                         };
#define USB_SOF_NUMBER() ((((USB_OTG_DeviceTypeDef *)((uint32_t )USB_OTG_FS + USB_OTG_DEVICE_BASE))->DSTS&USB_OTG_DSTS_FNSOF)>>USB_OTG_DSTS_FNSOF_Pos)

#define IS_ISO_IN_INCOMPLETE_EP(ep_addr,current_sof, transmit_soffn) ((USB_DIEPCTL(ep_addr)&USB_OTG_DIEPCTL_EPENA_Msk)&&\
                                                          (((current_sof&0x01) == ((USB_DIEPCTL(ep_addr)&USB_OTG_DIEPCTL_EONUM_DPID_Msk)>>USB_OTG_DIEPCTL_EONUM_DPID_Pos))\
                                                            ||(current_sof== ((transmit_soffn+2)&0x7FF))))

#define AUDIO_SAMPLE_FREQ(frq) (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

/*
 * Max packet size: (freq / 1000 + extra_samples) * channels * bytes_per_sample
 * e.g. (48000 / 1000 + 1) * 2(stereo) * 2(16bit) = 196
 */
#define AUDIO_PACKET_SZE_16B(frq) (uint8_t)(((frq / 1000U + 4U) * 2U * 2U) & 0xFFU), \
                                  (uint8_t)((((frq / 1000U + 4U) * 2U * 2U) >> 8) & 0xFFU)

#define MIC_PACKET_SZE(frq)    (uint8_t)(((frq / 1000U + 1U) * 1U * 2U) & 0xFFU), \
                                  (uint8_t)((((frq / 1000U + 1U) * 1U * 2U) >> 8) & 0xFFU)

/* Feature Unit Config */
#define AUDIO_CONTROL_FEATURES AUDIO_CONTROL_MUTE | AUDIO_CONTROL_VOL

/**
 * The minimum distance that the wr_ptr should keep before rd_ptr to
 * prevent overwriting unplayed buffer
 */
#define AUDIO_BUF_SAFEZONE                            ((uint16_t)((USBD_AUDIO_FREQ / 1000U) * 2U * 4U))

    /* Audio Commands enumeration */
typedef enum
{
  AUDIO_CMD_START = 1,
  AUDIO_CMD_PLAY,
  AUDIO_CMD_STOP,
  AUDIO_CMD_RECORD,
  AUDIO_CMD_RESUME,
}AUDIO_CMD_TypeDef;


typedef enum
{
  AUDIO_OFFSET_NONE = 0,
  AUDIO_OFFSET_HALF,
  AUDIO_OFFSET_FULL,
  AUDIO_OFFSET_UNKNOWN,
}
AUDIO_OffsetTypeDef;
/**
  * @}
  */


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
 typedef struct
{
   uint8_t cmd;                    /* bRequest */
   uint8_t req_type;               /* bmRequest */
   uint8_t cs;                     /* wValue (high byte): Control Selector */
   uint8_t cn;                     /* wValue (low byte): Control Number */
   uint8_t unit;                   /* wIndex: Feature Unit ID, Extension Unit ID, or Interface, Endpoint */
   uint8_t len;                    /* wLength */
   uint8_t data[USB_MAX_EP0_SIZE]; /* Data */
}
USBD_AUDIO_ControlTypeDef;



typedef struct
{
  uint32_t                  alt_setting[USBD_MAX_NUM_INTERFACES + 1];
  uint16_t*                 out_buffer;
  uint8_t* 					out_packet_buffer;
  uint8_t                   out_rd_enable;
  uint16_t                  out_rd_ptr;
  uint16_t                  out_wr_ptr;
  int16_t                   vol;
  /* mic data */
  uint16_t*                 in_buffer;
  uint8_t*					in_packet_buffer;
  uint16_t 					in_packet_size;
  uint8_t 					in_packet_buffer_enable;
  uint16_t                  in_rd_ptr;
  uint16_t                  in_wr_ptr;
  USBD_AUDIO_ControlTypeDef control;
}
USBD_AUDIO_HandleTypeDef;


typedef struct
{
    int8_t  (*Init)         (uint32_t  AudioFreq, uint32_t Volume, uint32_t options);
    int8_t  (*DeInit)       (uint32_t options);
    int8_t  (*AudioCmd)     (uint16_t* pbuf, uint32_t size, uint8_t cmd);
    int8_t  (*VolumeCtl)    (uint8_t vol);
    int8_t  (*MuteCtl)      (uint8_t cmd);
}USBD_AUDIO_ItfTypeDef;
/**
  * @}
  */



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef  USBD_AUDIO;
#define USBD_AUDIO_CLASS    &USBD_AUDIO
/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t  USBD_AUDIO_RegisterInterface  (USBD_HandleTypeDef   *pdev,
                                        USBD_AUDIO_ItfTypeDef *fops);

void USBD_AUDIO_UpdateBuffers(USBD_HandleTypeDef* pdev, AUDIO_OffsetTypeDef offset);
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_AUDIO_H */
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
