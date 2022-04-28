#include "keyboard.h"
#include "main.h"
#include "usbd_comp.h"

static void setFrontLedColor(uint32_t color);
static void setStateLedColor(StateLedColors color);
static void scanKeyboard(void);
static void StartTimer(void);

KeyboardState keyboardState;
KeyboardState *kbState;

uint8_t prev_kb_state = 0x3F;
uint8_t bt64bFwPacket[64] = {0xFF};

void initKeyboardState(void)
{
	  // init keyboard state struct
	  keyboardState.delayBetweenStimAndResponse = 0;
	  keyboardState.LED_state = 0;
	  keyboardState.isDelayMeasureTestEnabled = 0;
	  keyboardState.isScanningTimerUpdated = 0;
	  keyboardState.isBtFwUpdateStarted = 0;
	  keyboardState.isBtReadyToReceiveNextPacket = 1;
	  keyboardState.startBTBootMode = 0;
	  keyboardState.stopBTBootMode = 0;
	  keyboardState.btFwPacket64b = bt64bFwPacket;
	  keyboardState.StartDelayMeasureTimer = StartTimer;
	  keyboardState.SetFrontLedColor = setFrontLedColor;
	  keyboardState.SetStateLedColor = setStateLedColor;
	  keyboardState.ScanKeyboard = scanKeyboard;

	  kbState = &keyboardState;
}

static void setFrontLedColor(uint32_t color)
{

}

static void setStateLedColor(StateLedColors color)
{
	GPIOC->ODR &= 0xCFFF; // reset LED color
	GPIOC->ODR |= (color<<12);
}

static void scanKeyboard(void)
{
	uint8_t current_kb_state = (GPIOA->IDR & 0x3F);
	uint8_t pressed_keys_cntr = 2;
	uint8_t reportData[9] = {0};

	if(current_kb_state^prev_kb_state)
	{
		prev_kb_state = current_kb_state;

		current_kb_state ^= 0xFF; // invert current state data

		// prepare report data
		reportData[0] = 0x01; // report number
		// cancel button
		if((current_kb_state & KEY_1_MASK) && pressed_keys_cntr > 0)
		{
			pressed_keys_cntr--;
			reportData[2-pressed_keys_cntr] = 0xB1;
		}
		// ok button
		if((current_kb_state & KEY_2_MASK) && pressed_keys_cntr > 0)
		{
			pressed_keys_cntr--;
			reportData[2-pressed_keys_cntr] = 0xB2;
		}
		// cross button up
		if((current_kb_state & KEY_3_MASK) && pressed_keys_cntr > 0)
		{
			pressed_keys_cntr--;
			reportData[2-pressed_keys_cntr] = 0xA1;
		}
		// cross button down
		if((current_kb_state & KEY_4_MASK) && pressed_keys_cntr > 0)
		{
			pressed_keys_cntr--;
			reportData[2-pressed_keys_cntr] = 0xA2;
		}
		// cross button left
		if((current_kb_state & KEY_5_MASK) && pressed_keys_cntr > 0)
		{
			pressed_keys_cntr--;
			reportData[2-pressed_keys_cntr] = 0xA3;
		}
		// cross button left
		if((current_kb_state & KEY_6_MASK) && pressed_keys_cntr > 0)
		{
			pressed_keys_cntr--;
			reportData[2-pressed_keys_cntr] = 0xA4;
		}

		USBD_COMP_HID_SendReport_FS(reportData, sizeof(reportData)); // send report
	}
}

static void StartTimer(void)
{
	TIM7->CR1 |= TIM_CR1_CEN;
}
