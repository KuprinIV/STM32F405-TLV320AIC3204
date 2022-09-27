#ifndef __TLV320AIC3204_H
#define __TLV320AIC3204_H

#include "stm32f4xx_hal.h"

#define PAGE_SELECT_REGISTER 	0x00
#define DMA_MAX_SZE             0xFFFF
#define DMA_MAX(x)           	(((x) <= DMA_MAX_SZE)? (x):DMA_MAX_SZE)
#define CODEC_SLAVE_ADDRESS		0x30

typedef enum
{
	HEADPHONES = 0,
	LOUDSPEAKERS,
}OutputsType;

typedef enum
{
	MIC1 = 0,
	MIC3,
}InputsType;

typedef struct
{
	void (*PowerOnOff)(uint8_t is_powered);
	void (*InitInterface)(void);
	void (*CodecInit)(void);
	void (*DeInit)(void);
	void (*Reset)(void);
	void (*SelectOutput)(OutputsType output);
	void (*SelectInput)(InputsType input);
	void (*MuteCtrl)(uint8_t mute_state);
	void (*SetOutputDriverGain)(int8_t gain);
	void (*SetVolume)(int8_t volume);
	uint16_t (*GetOutDataRemainingSize)(void);
	uint16_t (*GetInDataRemainingSize)(void);
	void (*StartDataTransfer)(uint16_t* tx_data, uint16_t* rx_data, uint16_t size);
}AudioCodecDrv;

extern AudioCodecDrv *tlv320aic3204_drv;

#endif
