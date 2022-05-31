#include "keyboard.h"
#include "main.h"
#include "bt121.h"
#include "usbd_comp.h"
#include "tlv320aic3204.h"
#include "iir_filter.h"
#include "fir_filter.h"

static void setFrontLedColor(uint16_t pulse_length, uint32_t grb_color);
static void setStateLedColor(StateLedColors color);
static uint8_t scanKeyboard(void);
static void StartTimer(void);
static void calcJoystickCoords(JoystickData* jd, int8_t* x, int8_t* y);
static uint8_t isJoystickPositionChanged(JoystickData* jd);

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern ADC_HandleTypeDef hadc1;

KeyboardState keyboardState = {0, 0, 0, StartTimer, setFrontLedColor, setStateLedColor, scanKeyboard};
KeyboardState *kbState;

JoystickData joystickLeft = {3748, 366, 2056, 3731, 354, 2048, 2056, 2048}; // default values from schematic
JoystickData joystickRight = {3723, 380, 2046, 3724, 384, 2030, 2046, 2030}; // default values from schematic

#ifdef USE_IIR_FILTER
// init IIR filters data structs
IIR_FilterData iir_LP_50Hz_J1V = {2, 32, {10543, 21086, 10543}, {16384, -25575, 10507}, {2048, 2048, 2048}, {0, 0, 0}};
IIR_FilterData iir_LP_50Hz_J1H = {2, 32, {10543, 21086, 10543}, {16384, -25575, 10507}, {2048, 2048, 2048}, {0, 0, 0}};
IIR_FilterData iir_LP_50Hz_J2V = {2, 32, {10543, 21086, 10543}, {16384, -25575, 10507}, {2048, 2048, 2048}, {0, 0, 0}};
IIR_FilterData iir_LP_50Hz_J2H = {2, 32, {10543, 21086, 10543}, {16384, -25575, 10507}, {2048, 2048, 2048}, {0, 0, 0}};
IIR_FilterData iir_LP_50Hz_HPDET = {2, 32, {10543, 21086, 10543}, {16384, -25575, 10507}, {2048, 2048, 2048}, {0, 0, 0}};
#else
// init FIR filters data structs
FIR_FilterData fir_LP_25Hz_J1V = {524288, {2815, 4280, 5933, 7764, 9760, 11898, 14150, 16476, 18830, 21154, 23379, 25426, 27204,
		28611, 29530, 29853, 29530, 28611, 27204, 25426, 23379, 21154, 18830, 16476, 14150, 11898, 9760, 7764, 5933, 4280, 2815}, {0}};
FIR_FilterData fir_LP_25Hz_J1H = {524288, {2815, 4280, 5933, 7764, 9760, 11898, 14150, 16476, 18830, 21154, 23379, 25426, 27204,
		28611, 29530, 29853, 29530, 28611, 27204, 25426, 23379, 21154, 18830, 16476, 14150, 11898, 9760, 7764, 5933, 4280, 2815}, {0}};
FIR_FilterData fir_LP_25Hz_J2V = {524288, {2815, 4280, 5933, 7764, 9760, 11898, 14150, 16476, 18830, 21154, 23379, 25426, 27204,
		28611, 29530, 29853, 29530, 28611, 27204, 25426, 23379, 21154, 18830, 16476, 14150, 11898, 9760, 7764, 5933, 4280, 2815}, {0}};
FIR_FilterData fir_LP_25Hz_J2H = {524288, {2815, 4280, 5933, 7764, 9760, 11898, 14150, 16476, 18830, 21154, 23379, 25426, 27204,
		28611, 29530, 29853, 29530, 28611, 27204, 25426, 23379, 21154, 18830, 16476, 14150, 11898, 9760, 7764, 5933, 4280, 2815}, {0}};
FIR_FilterData fir_LP_25Hz_HPDET = {524288, {2815, 4280, 5933, 7764, 9760, 11898, 14150, 16476, 18830, 21154, 23379, 25426, 27204,
		28611, 29530, 29853, 29530, 28611, 27204, 25426, 23379, 21154, 18830, 16476, 14150, 11898, 9760, 7764, 5933, 4280, 2815}, {0}};
#endif

uint8_t prev_kb_state = 0x3F;
uint8_t bt64bFwPacket[64] = {0xFF};

uint16_t LEDs_fb[LEDS_COUNT+2][24] = {0}; // LEDs data framebuffer
uint16_t adcSamples[5] = {0}; // IN6 - J1_AV; IN7 - J1_AH; IN12 - HP_DET; IN14 - J2_AV; IN15 - J2_AH
uint16_t hp_detection_level = 4095;

void initKeyboardState(void)
{
	kbState = &keyboardState;
	// start ADC conversion
	HAL_TIM_Base_Start(&htim8);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcSamples, 5);
}

static void setFrontLedColor(uint16_t pulse_length, uint32_t grb_color)
{
	memset(LEDs_fb, 0, sizeof(LEDs_fb));
	// fill LEDs framebuffer
	for(uint8_t i = 0; i < LEDS_COUNT; i++)
	{
		for(uint8_t j = 0; j < 24; j++)
		{
			LEDs_fb[i][j] = (((grb_color>>(23-j)) & 0x01)) ? (BIT1) : (BIT0);
		}
	}
	// 50 us pause
	for(uint8_t i = LEDS_COUNT; i < LEDS_COUNT+2; i++)
	{
		for(uint8_t j = 0; j < 24; j++)
		{
			LEDs_fb[i][j] = 0;
		}
	}
	// start data transfer
	DMA1_Stream0->M0AR = (uint32_t)LEDs_fb;
	DMA1_Stream0->NDTR = (LEDS_COUNT+2)*24; // set burst data size
	DMA1_Stream0->CR |= DMA_SxCR_EN; // enable DMA

	TIM4->CR1 |= TIM_CR1_CEN; // TIM4 enable

	// start timer for limiting pulse length
	if(pulse_length > 0)
	{
		TIM5->ARR = pulse_length-1;
		TIM5->EGR |= TIM_EGR_UG; // call update event for setting actual ARR register value
		TIM5->CR1 |= TIM_CR1_CEN; // enable timer
		// start timer for delay test
		kbState->isDelayMeasureTestEnabled = 1;
		StartTimer();
	}
}

static void setStateLedColor(StateLedColors color)
{
	GPIOC->ODR &= 0xBFFF; // reset LED color
	GPIOC->ODR |= (color<<13);
}

static uint8_t scanKeyboard(void)
{
	uint8_t current_kb_state = (GPIOA->IDR & 0x3F);
	uint8_t pressed_keys_cntr = 2;
	uint8_t needToSendReport = 0;
	uint8_t isDelayTestButtonPressed = 0;
	uint8_t reportData[10] = {0};
	int8_t j1_h = 0;
	int8_t j1_v = 0;
	int8_t j2_h = 0;
	int8_t j2_v = 0;

	// prepare report data
	reportData[0] = 0x01; // report number

	// check keyboard state
	if(current_kb_state^prev_kb_state)
	{
		prev_kb_state = current_kb_state;
		needToSendReport = 1;
	}

	// form report data from pressed keys
	current_kb_state ^= 0xFF; // invert current state data
	// cancel button
	if((current_kb_state & KEY_1_MASK) && pressed_keys_cntr > 0)
	{
		pressed_keys_cntr--;
		reportData[2-pressed_keys_cntr] = KEY_CANCEL;
		isDelayTestButtonPressed = 1;
	}
	// ok button
	if((current_kb_state & KEY_2_MASK) && pressed_keys_cntr > 0)
	{
		pressed_keys_cntr--;
		reportData[2-pressed_keys_cntr] = KEY_OK;
		isDelayTestButtonPressed = 1;
	}
	// cross button up
	if((current_kb_state & KEY_3_MASK) && pressed_keys_cntr > 0)
	{
		pressed_keys_cntr--;
		reportData[2-pressed_keys_cntr] = KEY_UP;
	}
	// cross button down
	if((current_kb_state & KEY_4_MASK) && pressed_keys_cntr > 0)
	{
		pressed_keys_cntr--;
		reportData[2-pressed_keys_cntr] = KEY_DOWN;
	}
	// cross button left
	if((current_kb_state & KEY_5_MASK) && pressed_keys_cntr > 0)
	{
		pressed_keys_cntr--;
		reportData[2-pressed_keys_cntr] = KEY_LEFT;
	}
	// cross button left
	if((current_kb_state & KEY_6_MASK) && pressed_keys_cntr > 0)
	{
		pressed_keys_cntr--;
		reportData[2-pressed_keys_cntr] = KEY_RIGHT;
	}

	// send delay test result data
	if(isDelayTestButtonPressed)
	{
	  if(kbState->isDelayMeasureTestEnabled && (TIM7->CR1 & TIM_CR1_CEN))
	  {
		  kbState->delayBetweenStimAndResponse = TIM7->CNT;

		  reportData[3] = (uint8_t)(kbState->delayBetweenStimAndResponse>>8); // measured delay MSB
		  reportData[4] = (uint8_t)(kbState->delayBetweenStimAndResponse & 0xFF); // measured delay LSB
		  TIM7->EGR |= TIM_EGR_UG; // call update event for disabling timer
	  }
	}

	// check joysticks position
	if(isJoystickPositionChanged(&joystickLeft) || isJoystickPositionChanged(&joystickRight) || needToSendReport)
	{
		// calculate joysticks position
		calcJoystickCoords(&joystickLeft, &j1_h, &j1_v);
		calcJoystickCoords(&joystickRight, &j2_h, &j2_v);

		// add joysticks position to report data
		reportData[5] = (uint8_t)j1_h;
		reportData[6] = (uint8_t)j1_v;
		reportData[7] = (uint8_t)j2_h;
		reportData[8] = (uint8_t)j2_v;

		needToSendReport = 1;
	}

	// get battery charge value
	reportData[9] = 100; // TODO: add real data

	if(needToSendReport)
	{
		// send report via USB
		USBD_COMP_HID_SendReport_FS(reportData, sizeof(reportData));
		//send report via Bluetooth, if it is connected to host
		if(bt121_drv->IsHID_EndpointConnected())
		{
			bt121_drv->SendInputReport(reportData[0], &reportData[1], sizeof(reportData)-1);
		}
	}

	return needToSendReport;
}

static void StartTimer(void)
{
	TIM7->CR1 |= TIM_CR1_CEN;
}

static void calcJoystickCoords(JoystickData* jd, int8_t* x, int8_t* y)
{
	// calculate x-coordinate
	if(jd->h_value >= jd->h_zero)
	{
		*x = (int8_t)((float)(jd->h_value - jd->h_zero)*127/(jd->h_max - jd->h_zero));
	}
	else
	{
		*x = (int8_t)((float)(jd->h_value - jd->h_zero)*127/(jd->h_zero - jd->h_min));
	}
	// check limits
	if(*x > 127) *x = 127;
	if(*x < -127) *x = -127;

	// calculate y-coordinate
	if(jd->v_value >= jd->v_zero)
	{
		*y = (int8_t)((float)(jd->v_value - jd->v_zero)*127/(jd->v_max - jd->v_zero));
	}
	else
	{
		*y = (int8_t)((float)(jd->v_value - jd->v_zero)*127/(jd->v_zero - jd->v_min));
	}
	// check limits
	if(*y > 127) *y = 127;
	if(*y < -127) *y = -127;
}

static uint8_t isJoystickPositionChanged(JoystickData* jd)
{
	uint8_t res = 0;

	if(jd->h_value > jd->h_value_prev + MIN_JOYSTICK_DELTA || jd->h_value < jd->h_value_prev - MIN_JOYSTICK_DELTA
			|| jd->v_value > jd->v_value_prev + MIN_JOYSTICK_DELTA || jd->v_value < jd->v_value_prev - MIN_JOYSTICK_DELTA)
	{
		res = 1;

		jd->h_value_prev = jd->h_value;
		jd->v_value_prev = jd->v_value;
	}

	return res;
}

// get analog channels data
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	UNUSED(hadc);

	static uint16_t hp_detection_level_prev = 4095;

#ifdef USE_IIR_FILTER
	joystickLeft.h_value = doIirFilter(&iir_LP_50Hz_J1H, (int16_t)adcSamples[1]);
	joystickLeft.v_value = doIirFilter(&iir_LP_50Hz_J1V, (int16_t)adcSamples[0]);

	joystickRight.h_value = doIirFilter(&iir_LP_50Hz_J2H, (int16_t)adcSamples[4]);
	joystickRight.v_value = doIirFilter(&iir_LP_50Hz_J2V, (int16_t)adcSamples[3]);

	hp_detection_level = doIirFilter(&iir_LP_50Hz_HPDET, (int16_t)adcSamples[2]);
#else
	joystickLeft.h_value = doFirFilter(&fir_LP_25Hz_J1H, (int16_t)adcSamples[1]);
	joystickLeft.v_value = doFirFilter(&fir_LP_25Hz_J1V, (int16_t)adcSamples[0]);

	joystickRight.h_value = doFirFilter(&fir_LP_25Hz_J2H, (int16_t)adcSamples[4]);
	joystickRight.v_value = doFirFilter(&fir_LP_25Hz_J2V, (int16_t)adcSamples[3]);

	hp_detection_level = doFirFilter(&fir_LP_25Hz_HPDET, (int16_t)adcSamples[2]);
#endif

	// headphones is connected, switch codec's output to headphones
	if(hp_detection_level_prev >= HEADPHONES_DETECTION_THRESHOLD_LEVEL && hp_detection_level < HEADPHONES_DETECTION_THRESHOLD_LEVEL)
	{
		// switch codec's output to headphones
		tlv320aic3204_drv->SelectOutput(HEADPHONES);
	}
	// headphones is disconnected, switch codec's output to loudspeakers
	if(hp_detection_level_prev < HEADPHONES_DETECTION_THRESHOLD_LEVEL && hp_detection_level >= HEADPHONES_DETECTION_THRESHOLD_LEVEL)
	{
		// switch codec's output to loudspeakers
		tlv320aic3204_drv->SelectOutput(LOUDSPEAKERS);
	}
	// update previous level
	hp_detection_level_prev = hp_detection_level;
}
