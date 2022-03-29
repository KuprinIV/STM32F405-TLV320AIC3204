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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
KeyboardState keyboardState;
KeyboardState *kbState;
uint8_t bt64bFwPacket[64] = {0xFF};
uint8_t btFwDataPacket[BT_FW_PACKET_SIZE] = {0xFF};
uint8_t outBootStateData[2] = {0x07, 0x00};
uint8_t packet_64b_cntr = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void MX_Timers_Init(void);
static void StartTimer(void);
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

  // init keyboard state struct
  keyboardState.delayBetweenStimAndResponse = 0;
  keyboardState.StartDelayMeasureTimer = StartTimer;
  keyboardState.LED_state = 0;
  keyboardState.isDelayMeasureTestEnabled = 0;
  keyboardState.isScanningTimerUpdated = 0;
  keyboardState.isBtFwUpdateStarted = 0;
  keyboardState.isBtReadyToReceiveNextPacket = 1;
  keyboardState.startBTBootMode = 0;
  keyboardState.stopBTBootMode = 0;
  keyboardState.btFwPacket64b = bt64bFwPacket;

  kbState = &keyboardState;
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
  tlv320aic3204_drv->Reset();
  MX_USART2_UART_Init();
  MX_Timers_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  bt121_drv->Init();
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
		  if(((USER_BUTTON_GPIO_Port->IDR & USER_BUTTON_Pin) == 0) && !isButtonPressed) // if button pressed
		  {
			  isButtonPressed = 1;
			  btSleepCounter = 0; // reset BT sleep mode counter
			  bt121_drv->SetEnabled(1); // enable BT
			  if(kbState->isDelayMeasureTestEnabled && (TIM7->CR1 & TIM_CR1_CEN))
			  {
				  kbState->delayBetweenStimAndResponse = TIM7->CNT;

				  report_data[0] = 0x02;
				  report_data[1] = (uint8_t)(kbState->delayBetweenStimAndResponse>>8); // measured delay MSB
				  report_data[2] = (uint8_t)(kbState->delayBetweenStimAndResponse & 0xFF); // measured delay LSB
				  TIM7->EGR |= TIM_EGR_UG;
				  USBD_COMP_HID_SendReport_FS(report_data, 3); // send report
			  }
			  if(bt121_drv->IsHID_EndpointConnected())
			  {
				  report_data[0] = 0x01; // report ID
				  report_data[1] = 0xB2;
				  report_data[7] = 0x53;
				  bt121_drv->SendInputReport(report_data[0], &report_data[1], sizeof(report_data)-1);
			  }
		  }
		  else if(((USER_BUTTON_GPIO_Port->IDR & USER_BUTTON_Pin) == 0) && isButtonPressed) // if button is pressed yet
		  {
			  btSleepCounter = 0; // reset BT sleep mode counter
			  if(!isLongPressDetected)
			  {
				  if(longPressCntr == 39)
				  {
					  isButtonLongPressed = 1;
					  isLongPressDetected = 1;
				  }
				  else
				  {
					  longPressCntr++;
				  }
			  }
		  }
		  else if((USER_BUTTON_GPIO_Port->IDR & USER_BUTTON_Pin) && isButtonPressed) // if button released
		  {
			  isButtonPressed = 0;
			  longPressCntr = 0;
			  isButtonLongPressed = 0;
			  isLongPressDetected = 0;
			  btSleepCounter = 0; // reset BT sleep mode counter

			  if(bt121_drv->IsHID_EndpointConnected())
			  {
				  report_data[0] = 0x01; // report ID
				  report_data[1] = 0x00;
				  report_data[7] = 0x53;
				  bt121_drv->SendInputReport(report_data[0], &report_data[1], sizeof(report_data)-1);
			  }
		  }
		  // handle long button press action
		  if(isButtonLongPressed)
		  {
			  isButtonLongPressed = 0;
			  bt121_drv->DeleteBonding();
		  }
		  // check BT module to sleep
		  if(btSleepCounter < BT_SLEEP_DELAY)
		  {
			  btSleepCounter++;
		  }
		  else
		  {
			  bt121_drv->SetEnabled(0); // disable BT
		  }
	  }
	  // set BT to boot mode
	  if(kbState->startBTBootMode)
	  {
		  kbState->startBTBootMode = 0;
		  res = bt121_drv->BootModeCtrl(1); // BT boot mode control
		  if(res == HAL_OK)
		  {
			  res = bt121_drv->BT_FlashErase();
			  if(res == HAL_OK)
			  {
				  outBootStateData[1] = 0x7F;
				  btStartFlashAddr = BT_START_FLASH_ADDR;
			  }
			  else
			  {
				  outBootStateData[1] = res;
			  }
		  }
		  else
		  {
			  outBootStateData[1] = res;
		  }
		  USBD_COMP_HID_SendReport_FS(outBootStateData, 2); // send transfer result of firmware packet to BT
	  }
	  // exit BT from boot mode
	  if(kbState->stopBTBootMode)
	  {
		  kbState->stopBTBootMode = 0;
		  kbState->isBtFwUpdateStarted = 0;
		  btStartFlashAddr = BT_START_FLASH_ADDR;
		  bt121_drv->BootModeCtrl(0); // BT boot mode control

	  }
	  // handle BT firmware update data transfer
	  if(kbState->isBtFwUpdateStarted)
	  {
		  if(!kbState->isBtReadyToReceiveNextPacket)
		  {
			  kbState->isBtReadyToReceiveNextPacket = 1;

			  if(packet_64b_cntr < ((BT_FW_PACKET_SIZE>>6)-1))
			  {
				  memcpy(btFwDataPacket+(packet_64b_cntr<<6), kbState->btFwPacket64b, 64);
				  packet_64b_cntr++;

				  outBootStateData[0] = 0x07;
				  outBootStateData[1] = HAL_OK; // set success state
				  USBD_COMP_HID_SendReport_FS(outBootStateData, 2); // send transfer result of firmware packet to BT
			  }
			  else
			  {
				  memcpy(btFwDataPacket+(packet_64b_cntr<<6), kbState->btFwPacket64b, 64);
				  packet_64b_cntr = 0;

				  res = bt121_drv->BT_FlashWrite(btStartFlashAddr, btFwDataPacket, BT_FW_PACKET_SIZE);
				  if(res == HAL_OK)
				  {
					  // verify flash data
					  res = bt121_drv->BT_FlashVerify(btStartFlashAddr, btFwDataPacket, BT_FW_PACKET_SIZE);
					  if(res == HAL_OK)
					  {
						  btStartFlashAddr += BT_FW_PACKET_SIZE;

						  outBootStateData[0] = 0x07;
						  outBootStateData[1] = HAL_OK; // set success state
						  USBD_COMP_HID_SendReport_FS(outBootStateData, 2); // send transfer result of firmware packet to BT
					  }
					  else
					  {
						  outBootStateData[0] = 0x07;
						  outBootStateData[1] = res; // set error state, because error occurred during flash data verification
						  kbState->stopBTBootMode = 1;
						  USBD_COMP_HID_SendReport_FS(outBootStateData, 2); // send transfer result of firmware packet to BT
					  }
				  }
				  else
				  {
					  outBootStateData[0] = 0x07;
					  outBootStateData[1] = res; // set error state, because error occurred during flash write
					  kbState->stopBTBootMode = 1;
					  USBD_COMP_HID_SendReport_FS(outBootStateData, 2); // send transfer result of firmware packet to BT
				  }
			  }
		  }
	  }
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BT_RST_Pin|BT_BOOT_Pin|CODEC_RST_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_SS_GPIO_Port, SPI1_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : USER_BUTTON_Pin BT_LINK_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin|BT_LINK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BT_RST_Pin BT_BOOT_Pin CODEC_RST_Pin LED_Pin */
  GPIO_InitStruct.Pin = BT_RST_Pin|BT_BOOT_Pin|CODEC_RST_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_SS_Pin */
  GPIO_InitStruct.Pin = SPI1_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_SS_GPIO_Port, &GPIO_InitStruct);

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

static void StartTimer(void)
{
	TIM7->CR1 |= TIM_CR1_CEN;
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

