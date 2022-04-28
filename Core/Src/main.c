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
#include "usbd_comp.h"
#include "bt121.h"
#include "keyboard.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BT_SLEEP_DELAY 2400 // 60 sec (in 25 ms steps)
#define BT_FW_PACKET_SIZE 128
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

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim4_up;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t btFwDataPacket[BT_FW_PACKET_SIZE] = {0xFF};
uint8_t outBootStateData[2] = {0x07, 0x00};
uint8_t packet_64b_cntr = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
static void MX_Timers_Init(void);
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
  uint8_t isButtonPressed = 0;
  uint8_t isButtonLongPressed = 0;
  uint8_t isLongPressDetected = 0;
  uint8_t report_data[9] = {0};
  uint8_t longPressCntr = 0;
  uint8_t res;
  uint16_t btSleepCounter = 0;
  uint32_t btStartFlashAddr = BT_START_FLASH_ADDR;
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
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  bt121_drv->Init();
  initKeyboardState();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(kbState->isScanningTimerUpdated)
	  {
		  kbState->isScanningTimerUpdated = 0;
//		  if(((USER_BUTTON_GPIO_Port->IDR & USER_BUTTON_Pin) == 0) && !isButtonPressed) // if button pressed
//		  {
//			  isButtonPressed = 1;
//			  btSleepCounter = 0; // reset BT sleep mode counter
//			  bt121_drv->SetEnabled(1); // enable BT
//			  if(kbState->isDelayMeasureTestEnabled && (TIM7->CR1 & TIM_CR1_CEN))
//			  {
//				  kbState->delayBetweenStimAndResponse = TIM7->CNT;
//
//				  report_data[0] = 0x02;
//				  report_data[1] = (uint8_t)(kbState->delayBetweenStimAndResponse>>8); // measured delay MSB
//				  report_data[2] = (uint8_t)(kbState->delayBetweenStimAndResponse & 0xFF); // measured delay LSB
//				  TIM7->EGR |= TIM_EGR_UG;
//				  USBD_COMP_HID_SendReport_FS(report_data, 3); // send report
//			  }
//			  if(bt121_drv->IsHID_EndpointConnected())
//			  {
//				  report_data[0] = 0x01; // report ID
//				  report_data[1] = 0xB2;
//				  report_data[7] = 0x53;
//				  bt121_drv->SendInputReport(report_data[0], &report_data[1], sizeof(report_data)-1);
//			  }
//		  }
//		  else if(((USER_BUTTON_GPIO_Port->IDR & USER_BUTTON_Pin) == 0) && isButtonPressed) // if button is pressed yet
//		  {
//			  btSleepCounter = 0; // reset BT sleep mode counter
//			  if(!isLongPressDetected)
//			  {
//				  if(longPressCntr == 39)
//				  {
//					  isButtonLongPressed = 1;
//					  isLongPressDetected = 1;
//				  }
//				  else
//				  {
//					  longPressCntr++;
//				  }
//			  }
//		  }
//		  else if((USER_BUTTON_GPIO_Port->IDR & USER_BUTTON_Pin) && isButtonPressed) // if button released
//		  {
//			  isButtonPressed = 0;
//			  longPressCntr = 0;
//			  isButtonLongPressed = 0;
//			  isLongPressDetected = 0;
//			  btSleepCounter = 0; // reset BT sleep mode counter
//
//			  if(bt121_drv->IsHID_EndpointConnected())
//			  {
//				  report_data[0] = 0x01; // report ID
//				  report_data[1] = 0x00;
//				  report_data[7] = 0x53;
//				  bt121_drv->SendInputReport(report_data[0], &report_data[1], sizeof(report_data)-1);
//			  }
//		  }
//		  // handle long button press action
//		  if(isButtonLongPressed)
//		  {
//			  isButtonLongPressed = 0;
//			  bt121_drv->DeleteBonding();
//		  }
//		  // check BT module to sleep
//		  if(btSleepCounter < BT_SLEEP_DELAY)
//		  {
//			  btSleepCounter++;
//		  }
//		  else
//		  {
//			  bt121_drv->SetEnabled(0); // disable BT
//		  }
		  kbState->ScanKeyboard();
	  }
	  // set BT to boot mode
//	  if(kbState->startBTBootMode)
//	  {
//		  kbState->startBTBootMode = 0;
//		  res = bt121_drv->BootModeCtrl(1); // BT boot mode control
//		  if(res == HAL_OK)
//		  {
//			  res = bt121_drv->BT_FlashErase();
//			  if(res == HAL_OK)
//			  {
//				  outBootStateData[1] = 0x7F;
//				  btStartFlashAddr = BT_START_FLASH_ADDR;
//			  }
//			  else
//			  {
//				  outBootStateData[1] = res;
//			  }
//		  }
//		  else
//		  {
//			  outBootStateData[1] = res;
//		  }
//		  USBD_COMP_HID_SendReport_FS(outBootStateData, 2); // send transfer result of firmware packet to BT
//	  }
	  // exit BT from boot mode
//	  if(kbState->stopBTBootMode)
//	  {
//		  kbState->stopBTBootMode = 0;
//		  kbState->isBtFwUpdateStarted = 0;
//		  btStartFlashAddr = BT_START_FLASH_ADDR;
//		  bt121_drv->BootModeCtrl(0); // BT boot mode control
//
//	  }
	  // handle BT firmware update data transfer
//	  if(kbState->isBtFwUpdateStarted)
//	  {
//		  if(!kbState->isBtReadyToReceiveNextPacket)
//		  {
//			  kbState->isBtReadyToReceiveNextPacket = 1;
//
//			  if(packet_64b_cntr < ((BT_FW_PACKET_SIZE>>6)-1))
//			  {
//				  memcpy(btFwDataPacket+(packet_64b_cntr<<6), kbState->btFwPacket64b, 64);
//				  packet_64b_cntr++;
//
//				  outBootStateData[0] = 0x07;
//				  outBootStateData[1] = HAL_OK; // set success state
//				  USBD_COMP_HID_SendReport_FS(outBootStateData, 2); // send transfer result of firmware packet to BT
//			  }
//			  else
//			  {
//				  memcpy(btFwDataPacket+(packet_64b_cntr<<6), kbState->btFwPacket64b, 64);
//				  packet_64b_cntr = 0;
//
//				  res = bt121_drv->BT_FlashWrite(btStartFlashAddr, btFwDataPacket, BT_FW_PACKET_SIZE);
//				  if(res == HAL_OK)
//				  {
//					  // verify flash data
//					  res = bt121_drv->BT_FlashVerify(btStartFlashAddr, btFwDataPacket, BT_FW_PACKET_SIZE);
//					  if(res == HAL_OK)
//					  {
//						  btStartFlashAddr += BT_FW_PACKET_SIZE;
//
//						  outBootStateData[0] = 0x07;
//						  outBootStateData[1] = HAL_OK; // set success state
//						  USBD_COMP_HID_SendReport_FS(outBootStateData, 2); // send transfer result of firmware packet to BT
//					  }
//					  else
//					  {
//						  outBootStateData[0] = 0x07;
//						  outBootStateData[1] = res; // set error state, because error occurred during flash data verification
//						  kbState->stopBTBootMode = 1;
//						  USBD_COMP_HID_SendReport_FS(outBootStateData, 2); // send transfer result of firmware packet to BT
//					  }
//				  }
//				  else
//				  {
//					  outBootStateData[0] = 0x07;
//					  outBootStateData[1] = res; // set error state, because error occurred during flash write
//					  kbState->stopBTBootMode = 1;
//					  USBD_COMP_HID_SendReport_FS(outBootStateData, 2); // send transfer result of firmware packet to BT
//				  }
//			  }
//		  }
//	  }
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
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 47;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  htim8.Init.Prescaler = 479;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 99;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_R_Pin|LED_G_Pin|CODEC_RST_Pin|AUD_EN_Pin
                          |BT_BOOT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LS_EN_GPIO_Port, LS_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BT_RST_GPIO_Port, BT_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_R_Pin LED_G_Pin CODEC_RST_Pin AUD_EN_Pin
                           BT_BOOT_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_G_Pin|CODEC_RST_Pin|AUD_EN_Pin
                          |BT_BOOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SB6_Pin SB5_Pin SB4_Pin SB3_Pin
                           SB2_Pin SB1_Pin */
  GPIO_InitStruct.Pin = SB6_Pin|SB5_Pin|SB4_Pin|SB3_Pin
                          |SB2_Pin|SB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LS_EN_Pin */
  GPIO_InitStruct.Pin = LS_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LS_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BT_RST_Pin */
  GPIO_InitStruct.Pin = BT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BT_RST_GPIO_Port, &GPIO_InitStruct);

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
// measuring delay between stimulus start and button press timer init (1 ms step)
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN; // enable TIM7 clock
	TIM7->PSC = 47999; // divide internal clock
	TIM7->ARR = 65535;
	TIM7->DIER |= TIM_DIER_UIE; // update interrupt enable
	TIM7->CR1 |= TIM_CR1_ARPE; // enable ARR register preload
	// configure interrupt
	HAL_NVIC_SetPriority(TIM7_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM7_IRQn);

// button scanning ang LED blinking timer init (period 50 ms)
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // enable TIM7 clock
	TIM6->PSC = 23999; // divide internal clock
	TIM6->ARR = 49;
	TIM6->DIER |= TIM_DIER_UIE; // update interrupt enable
	TIM6->CR1 |= TIM_CR1_ARPE|TIM_CR1_CEN; // enable ARR register preload
	// configure interrupt
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

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
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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

