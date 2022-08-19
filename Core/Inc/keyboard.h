#ifndef __KEYBOARD_H
#define __KEYBOARD_H

#include <stdint.h>

#define KEY_1_MASK 								0x01
#define KEY_2_MASK 								0x02
#define KEY_3_MASK 								0x04
#define KEY_4_MASK 								0x08
#define KEY_5_MASK 								0x10
#define KEY_6_MASK 								0x20

// key codes
#define KEY_CANCEL								0xB1
#define KEY_OK									0xB2
#define KEY_UP									0xA1
#define KEY_DOWN								0xA2
#define KEY_LEFT								0xA3
#define KEY_RIGHT								0xA4

// WS2812B LEDs count
#define LEDS_COUNT 								4
// digital "0" and "1" pulses length for WS2812B
#define BIT0 									19
#define BIT1 									40

#define HEADPHONES_DETECTION_THRESHOLD_LEVEL 	2500
#define MIN_JOYSTICK_DELTA						20

//#define USE_IIR_FILTER							1

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
	uint8_t isDfuModeEnabled;
	// functions
	void (*StartDelayMeasureTimer)(void);
	void (*SetFrontLedColor)(uint16_t pulse_length, uint32_t grb_color);
	void (*SetStateLedColor)(StateLedColors color);
	uint8_t (*ScanKeyboard)(void);
	void (*JoysticksCalibrationModeControl)(uint8_t is_enabled);
	void (*SaveJoysticksCalibrationData)(uint16_t* joystickLeftCD, uint16_t* joystickRightCD);
}KeyboardState;

typedef struct
{
	// horizontal axis calibration parameters
	uint16_t h_max;
	uint16_t h_min;
	uint16_t h_zero;
	// vertical axis calibration parameters
	uint16_t v_max;
	uint16_t v_min;
	uint16_t v_zero;
	// current position
	uint16_t h_value;
	uint16_t v_value;
	// previous position
	uint16_t h_value_prev;
	uint16_t v_value_prev;
}JoystickData;

extern KeyboardState *kbState;

void initKeyboardState(void);

#endif
