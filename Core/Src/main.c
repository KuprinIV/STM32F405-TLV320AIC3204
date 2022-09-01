/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tlv320aic3204.h"
#include "fir_filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */
// init FIR filters data structs
FIR_FilterData fir_LP_25Hz_MICDET = {524288, {2815, 4280, 5933, 7764, 9760, 11898, 14150, 16476, 18830, 21154, 23379, 25426, 27204,
		28611, 29530, 29853, 29530, 28611, 27204, 25426, 23379, 21154, 18830, 16476, 14150, 11898, 9760, 7764, 5933, 4280, 2815}, {2048}};
FIR_FilterData fir_LP_25Hz_HPDET = {524288, {2815, 4280, 5933, 7764, 9760, 11898, 14150, 16476, 18830, 21154, 23379, 25426, 27204,
		28611, 29530, 29853, 29530, 28611, 27204, 25426, 23379, 21154, 18830, 16476, 14150, 11898, 9760, 7764, 5933, 4280, 2815}, {2048}};

volatile uint16_t adcSamples[2] = {0}; // IN11 - MIC_DET; IN12 - HP_DET
volatile uint16_t hp_detection_level = 4095;
volatile uint16_t mic_detection_level = 4095;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
static void MX_Timers_Init(void);
static uint8_t IsInRange(uint16_t value, uint16_t min, uint16_t max);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();

  tlv320aic3204_drv->Reset();

  MX_Timers_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  // start ADC conversion
  HAL_TIM_Base_Start(&htim8);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcSamples, 2);

  // indicate work state
  LED_G_GPIO_Port->ODR |= LED_G_Pin;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 375;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 959;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 99;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LS_EN_Pin|LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CODEC_RST_GPIO_Port, CODEC_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LS_EN_Pin LED_G_Pin */
  GPIO_InitStruct.Pin = LS_EN_Pin|LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CODEC_RST_Pin */
  GPIO_InitStruct.Pin = CODEC_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CODEC_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
   GPIO_InitStruct.Pin = GPIO_PIN_15;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void MX_Timers_Init(void)
{
// SOF period measuring timer
	/* Необходимо по приходу сигнала SOF захватывать значение счетчика таймера 2 в регистр захвата, а сам счетчик таймера 2 - сбрасывать
	 В процедуре обработки SOF'а флаг захвата сбрасывается, а значение накапливается, для того, чтобы выдать значение feedback rate
	 Поскольку система имеет три независимых источника тактовой частоты - генератор MCLK, частота USB SOF от хоста и HSE PLL,
	 таймер 2 тактируется от частоты MCLK=12288 кГц. Такми образом, между SOFами
	 таймер 2 должен насчитывать примерно 12200-12400. Это значение должно попадать в регистр захвата таймера 2. Т.к. согласно стандарту
	 значение feedback value должно выдаваться в формате 10.14 и содержать отношение fs/fsof, а накопление идет 2^SOF_VALUE периодов,
	 получаем за период SOF - 12288 импульса
	 сдвинуть нужно на 6 разрядов влево, чтобы получить feedback_value
	 */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enable TIM2 clock
	TIM2->PSC = 0;
	TIM2->ARR = 0xFFFFFFFF;
	TIM2->SMCR |= (TIM_SMCR_ECE|TIM_SMCR_TS_0|TIM_SMCR_SMS_2); // enable Reset mode, select Internal Trigger 1 (ITR1).
	TIM2->CCMR1 |= TIM_CCMR1_CC1S; // CC1 channel is configured as input, IC1 is mapped on TRC
	TIM2->OR |= TIM_OR_ITR1_RMP_1; // OTG FS SOF is connected to the TIM2_ITR1 input
	TIM2->CCER |= TIM_CCER_CC1E; // Capture enabled
	TIM2->CR1 |= TIM_CR1_CEN; // Counter enabled
}

static uint8_t IsInRange(uint16_t value, uint16_t min, uint16_t max)
{
	if(value >= min && value <= max)
	{
		return 1;
	}
	return 0;
}

// get analog channels data
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	UNUSED(hadc);

	static uint16_t hp_detection_level_prev = 4095;
	static uint16_t mic_detection_level_prev = 4095;


	mic_detection_level = doFirFilter(&fir_LP_25Hz_MICDET, (int16_t)adcSamples[0]);
	hp_detection_level = doFirFilter(&fir_LP_25Hz_HPDET, (int16_t)adcSamples[1]);

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

	// mic from headset is connected, switch codec's input to MIC3
	if(!IsInRange(mic_detection_level_prev, MIC_DETECTION_THRESHOLD_LEVEL_MIN, MIC_DETECTION_THRESHOLD_LEVEL_MAX)
			&& IsInRange(mic_detection_level, MIC_DETECTION_THRESHOLD_LEVEL_MIN, MIC_DETECTION_THRESHOLD_LEVEL_MAX))
	{
		// switch codec's input to headset mic
		tlv320aic3204_drv->SelectInput(MIC3);
	}
	// mic from headset is disconnected, switch codec's input to MIC1
	if(IsInRange(mic_detection_level_prev, MIC_DETECTION_THRESHOLD_LEVEL_MIN, MIC_DETECTION_THRESHOLD_LEVEL_MAX)
			&& !IsInRange(mic_detection_level, MIC_DETECTION_THRESHOLD_LEVEL_MIN, MIC_DETECTION_THRESHOLD_LEVEL_MAX))
	{
		// switch codec's input to mic connector
		tlv320aic3204_drv->SelectInput(MIC1);
	}

	// update previous levels
	hp_detection_level_prev = hp_detection_level;
	mic_detection_level_prev = mic_detection_level;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

