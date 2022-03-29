#ifndef __TLV320AIC3204_H
#define __TLV320AIC3204_H

#include "stm32f4xx_hal.h"

#define PAGE_SELECT_REGISTER 0x00
#define DMA_MAX_SZE                          0xFFFF
#define DMA_MAX(x)           (((x) <= DMA_MAX_SZE)? (x):DMA_MAX_SZE)

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
	void (*InitInterface)(uint8_t dir);
	void (*PlaybackInit)(void);
	void (*RecordingInit)(void);
	void (*DeInit)(void);
	void (*Reset)(void);
	void (*SelectOutput)(OutputsType output);
	void (*SelectInput)(InputsType input);
	void (*MuteCtrl)(uint8_t mute_state);
	void (*SetOutputDriverGain)(int8_t gain);
	void (*SetVolume)(int8_t volume);
	void (*WriteData)(uint16_t* buffer, uint16_t size);
	void (*ReadData)(uint16_t* buffer, uint16_t size);
	void (*Stop)(void);
	void (*Resume)(void);
	uint16_t (*GetOutDataRemainingSize)(void);
	uint16_t (*GetInDataRemainingSize)(void);
	void (*SetFrequencyDeviation)(uint8_t dev_type);
	uint8_t (*isOutMuted)(void);
}AudioCodecDrv;

extern AudioCodecDrv *tlv320aic3204_drv;

#endif
