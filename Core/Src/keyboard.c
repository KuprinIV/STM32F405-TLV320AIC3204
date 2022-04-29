#include "keyboard.h"
#include "main.h"
#include "usbd_comp.h"

static void setFrontLedColor(uint32_t color);
static void setStateLedColor(StateLedColors color);
static void scanKeyboard(void);
static void StartTimer(void);

extern TIM_HandleTypeDef htim4;

KeyboardState keyboardState;
KeyboardState *kbState;

uint8_t prev_kb_state = 0x3F;
uint8_t bt64bFwPacket[64] = {0xFF};
volatile uint16_t LEDs_fb[LEDS_COUNT][24] = {0}; // LEDs data framebuffer

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

static void setFrontLedColor(uint32_t color_grb)
{
	// fill LEDs framebuffer
	for(uint8_t i = 0; i < LEDS_COUNT; i++)
	{
		for(uint8_t j = 0; j < 24; j++)
		{
			LEDs_fb[i][j] = (((color_grb>>(23-j)) & 0x01)) ? (BIT1) : (BIT0);
		}
	}
	// start data transfer
	DMA1_Stream0->M0AR = (uint32_t)LEDs_fb;
	DMA1_Stream0->NDTR = LEDS_COUNT*24; // set burst data size
	DMA1_Stream0->CR |= DMA_SxCR_EN; // enable DMA

	TIM4->CCMR1 |= TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1M_1; // set PWM mode 1
	TIM4->CR1 |= TIM_CR1_CEN; // TIM4 enable
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
