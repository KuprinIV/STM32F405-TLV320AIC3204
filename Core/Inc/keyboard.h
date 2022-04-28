#ifndef __KEYBOARD_H
#define __KEYBOARD_H

#include <stdint.h>

#define KEY_1_MASK 0x01
#define KEY_2_MASK 0x02
#define KEY_3_MASK 0x04
#define KEY_4_MASK 0x08
#define KEY_5_MASK 0x10
#define KEY_6_MASK 0x20

typedef enum
{
	NONE = 0x00,
	RED,
	GREEN,
	YELLOW,
}StateLedColors;

typedef struct
{
	uint8_t isDelayMeasureTestEnabled;
	uint16_t delayBetweenStimAndResponse;
	uint8_t isScanningTimerUpdated;
	uint8_t LED_state; // LED state: 0 - off, 1 - on, 2 - blinking 1 Hz, 3 - blinking 5 Hz
	// BT firmware update
	uint8_t isBtFwUpdateStarted;
	uint8_t isBtReadyToReceiveNextPacket;
	uint8_t startBTBootMode;
	uint8_t stopBTBootMode;
	uint8_t* btFwPacket64b;
	// functions
	void (*StartDelayMeasureTimer)(void);
	void (*SetFrontLedColor)(uint32_t grb_color);
	void (*SetStateLedColor)(StateLedColors color);
	void (*ScanKeyboard)(void);
}KeyboardState;

extern KeyboardState *kbState;

void initKeyboardState(void);

#endif
