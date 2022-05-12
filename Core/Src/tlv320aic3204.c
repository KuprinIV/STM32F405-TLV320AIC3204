#include "tlv320aic3204.h"
#include "main.h"
#include "usbd_comp.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_i2s2_ext_rx;
DMA_HandleTypeDef hdma_spi2_tx;

I2C_HandleTypeDef hi2c2;

uint8_t test = 0;

static void writeRegister(uint8_t addr, uint8_t value);
static uint8_t readRegister(uint8_t addr);

/**
 * @brief
 * Power control of audio part
 * @params
 * is_powered - 0 - power off, 1 - power on
 */
static void tlv320aic3204_PowerOnOff(uint8_t is_powered);

/**
 * @brief
 * Make codec hardware reset function
 */
static void tlv320aic3204_hardwareReset(void);

/**
 * @brief
 * Codec I2S interface init function
 */
static void tlv320aic3204_InterfaceInit(void);

/**
 * @brief
 * Codec playback and recording init function
 */
static void tlv320aic3204_CodecInit(void);

/**
 * @brief
 * Codec de-initialization function
 */
static void tlv320aic3204_DeInit(void);

/**
 * @brief
 * Select codec outputs
 * @params
 * outputs - select output channel (headphones or loudspeakers)
 */
static void tlv320aic3204_selectOutputs(OutputsType outputs);

/**
 * @brief
 * Select codec input
 * @params
 * input - selected input channel (IN1 or IN3)
 */
static void tlv320aic3204_selectInput(InputsType input);

/**
 * @brief
 * Set codec's output mute state
 * @params
 * is_enable - (0 - not muted, 1 - muted)
 */
static void tlv320aic3204_muteControl(uint8_t is_enabled);

/**
 * @brief
 * Set codec's output driver gain
 * @params
 * gain - value from -6 dB to 29 dB
 */
static void tlv320aic3204_setOutDriverGain(int8_t gain);

/**
 * @brief
 * Set codec's DAC output volume
 * @params
 * gain - value from -63,5 dB to 24 dB in discrets with 0,5 dB
 */
static void tlv320aic3204_setDigitalDACVolume(int8_t volume);

/**
 * @brief
 * Set internal codec's LDO state
 * @params
 * is_enabled - LDO state (0 - power down, 1 - power up)
 */
static void tlv320aic3204_LDO_PowerCtrl(uint8_t is_enabled);

/**
 * @brief
 * DAC test function: beep 1 kHz 5 sec
 */
//static void tlv320aic3204_BeepTest(void);

/**
 * @brief
 * Get I2S DMA remaining size for transmit
 */
static uint16_t tlv320aic3204_getOutRemainingDataSize(void);

/**
 * @brief
 * Get I2S DMA remaining size for receive
 */
static uint16_t tlv320aic3204_getInRemainingDataSize(void);

/**
 * @brief
 * Start data transfer with codec via I2S DMA full-duplex interface
 * @params
 * tx_data - transmit data buffer
 * rx_data - receive data buffer
 * size - data buffers length
 */
static void tlv320aic3204_StartDataTransfer(uint16_t* tx_data, uint16_t* rx_data, uint16_t size);

AudioCodecDrv tlv320aic3204_driver =
{
		tlv320aic3204_PowerOnOff,
		tlv320aic3204_InterfaceInit,
		tlv320aic3204_CodecInit,
		tlv320aic3204_DeInit,
		tlv320aic3204_hardwareReset,
		tlv320aic3204_selectOutputs,
		tlv320aic3204_selectInput,
		tlv320aic3204_muteControl,
		tlv320aic3204_setOutDriverGain,
		tlv320aic3204_setDigitalDACVolume,
		tlv320aic3204_getOutRemainingDataSize,
		tlv320aic3204_getInRemainingDataSize,
		tlv320aic3204_StartDataTransfer,
};

AudioCodecDrv *tlv320aic3204_drv = &tlv320aic3204_driver;
OutputsType currentOutputs = LOUDSPEAKERS;
InputsType currentInput = MIC1;
uint8_t interface_dir = 0; // output

static void writeRegister(uint8_t addr, uint8_t value)
{
	uint8_t data[2] = {0};
	data[0] = addr;
	data[1] = value;

	HAL_I2C_Master_Transmit(&hi2c2, 0x30, data, 2, 1000);
}

static uint8_t readRegister(uint8_t addr)
{
	uint8_t txData = 0;
	uint8_t rxData = 0;

	txData = addr;

	HAL_I2C_Master_Transmit(&hi2c2, 0x30, &txData, 1, 1000); // send register address
	HAL_I2C_Master_Receive(&hi2c2, 0x30, &rxData, 1, 1000); // receive register value
	return rxData;
}

static void tlv320aic3204_PowerOnOff(uint8_t is_powered)
{
	if(is_powered)
	{
		AUD_EN_GPIO_Port->ODR |= AUD_EN_Pin; // enable audio part power supply
	}
	else
	{
		AUD_EN_GPIO_Port->ODR &= ~AUD_EN_Pin; // disable audio part power supply
	}
}

static void tlv320aic3204_hardwareReset(void)
{
	HAL_Delay(2);
//	make hardware reset (pin PB10)
	CODEC_RST_GPIO_Port->BSRR = CODEC_RST_Pin<<16;
	HAL_Delay(1);
	CODEC_RST_GPIO_Port->BSRR = CODEC_RST_Pin;

//	wait analog and PLL startup
	HAL_Delay(2);
}

static void tlv320aic3204_InterfaceInit(void)
{
// codec control interface init
	  /* I2C2 parameter configuration*/
	  hi2c2.Instance = I2C2;
	  hi2c2.Init.ClockSpeed = 400000;
	  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
	  hi2c2.Init.OwnAddress1 = 0;
	  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	  hi2c2.Init.OwnAddress2 = 0;
	  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	  {
	    Error_Handler();
	  }
	
	/* DMA controller clock enable */
	  __HAL_RCC_DMA1_CLK_ENABLE();
// codec I2S interface init
	  hi2s2.Instance = SPI2;
	  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
	  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
	  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
	  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
	  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
	  hi2s2.Init.CPOL = I2S_CPOL_LOW;
	  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
	  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
	  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
	  {
	    Error_Handler();
	  }

	/* DMA interrupt init */
	  /* DMA1_Stream3_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	  /* DMA1_Stream4_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

static void tlv320aic3204_CodecInit(void)
{
// codec DAC init
	//select Page 0
	writeRegister(PAGE_SELECT_REGISTER, 0);
	// make software reset
	writeRegister(0x01, 0x01);
	// Power up the NDAC divider with value 1
	writeRegister(0x0B, 0x81);
	// Power up the MDAC divider with value 2
	writeRegister(0x0C, 0x82);
	// Program the OSR of DAC to 128
	writeRegister(0x0D, 0x00);
	writeRegister(0x0E, 0x80);
	// Set the word length of Audio Interface to 16bits PTM_P4
	writeRegister(0x1B, 0x00);
	// Set the DAC Mode to PRB_P8
	writeRegister(0x3C, 0x08);
	// Select Page 1
	writeRegister(PAGE_SELECT_REGISTER, 0x01);
	// Disable Internal Crude AVdd in presence of external AVdd supply or before
	//powering up internal AVdd LDO
	writeRegister(0x01, 0x08);
	// Enable Master Analog Power Control
	writeRegister(0x02, 0x01);
	// Set the REF charging time to 40ms
	writeRegister(0x7B, 0x01);
	// HP soft stepping settings for optimal pop performance at power up
	// Rpop used is 6k with N = 6 and soft step = 20usec. This should work with 47uF coupling
	// capacitor. Can try N=5,6 or 7 time constants as well. Trade-off delay vs “pop” sound.
	writeRegister(0x14, 0x25);
	// Set the Input Common Mode to 0.9V and Output Common Mode for Headphone to
	// Input Common Mode
	writeRegister(0x0A, 0x00);
	// Route Left DAC to LOL
	writeRegister(0x0E, 0x08);
	// Route Right DAC to LOR
	writeRegister(0x0F, 0x08);
	// Set the DAC PTM mode to PTM_P3/4
	writeRegister(0x03, 0x00);
	writeRegister(0x04, 0x00);
	// Set the HPL gain to 0dB
	writeRegister(0x10, 0x00);
	// Set the HPR gain to 0dB
	writeRegister(0x11, 0x00);
	// Set the LOL gain to 0dB
	writeRegister(0x12, 0x00);
	// Set the LOR gain to 0dB
	writeRegister(0x13, 0x00);
	// Power up LOL and LOR drivers
	writeRegister(0x09, 0x0C);
	// Wait for 2.5 sec for soft stepping to take effect
	// Else read Page 1, Register 63d, D(7:6). When = “11” soft-stepping is complete
	// Select Page 0
	writeRegister(PAGE_SELECT_REGISTER, 0);
	// Power up the Left and Right DAC Channels with route the Left Audio digital data to
	// Left Channel DAC and Right Audio digital data to Right Channel DAC, soft-step volume change enable
	writeRegister(0x3F, 0xD5);
	test = readRegister(0x3F);
	// Unmute the DAC digital volume control
	writeRegister(0x40, 0x00);
	// enable out amplifier for loudspeakers
	LS_EN_GPIO_Port->ODR |= LS_EN_Pin;

//codec ADC init
	// Initialize to Page 0
	writeRegister(PAGE_SELECT_REGISTER, 0);
	// Power up NADC divider with value 1
	writeRegister(0x12, 0x81);
	// Power up MADC divider with value 2
	writeRegister(0x13, 0x82);
	// Program OSR for ADC to 128
	writeRegister(0x14, 0x80);
	// Select ADC PRB_R1
	writeRegister(0x3B, 0x01);
	// Select Page 1
	writeRegister(PAGE_SELECT_REGISTER, 1);
	// Select ADC PTM_R4
	writeRegister(0x3D, 0x00);
	// Set MicPGA startup delay to 3.1ms
	writeRegister(0x47, 0x32);
	// Route IN1L to LEFT_P with 20K input impedance
	writeRegister(0x34, 0x80);
	// Route Common Mode to LEFT_M with impedance of 20K
	writeRegister(0x36, 0x80);
	// Route IN1R to RIGHT_P with input impedance of 20K
	writeRegister(0x37, 0x80);
	// Route Common Mode to RIGHT_M with impedance of 20K
	writeRegister(0x39, 0x80);
	// Unmute Left MICPGA, Gain selection of 6dB to make channel gain 0dB
	// Register of 6dB with input impedance of 20K => Channel Gain of 0dB
	writeRegister(0x3B, 0x0C);
	// Unmute Right MICPGA, Gain selection of 6dB to make channel gain 0dB
	// Register of 6dB with input impedance of 20K => Channel Gain of 0dB
	writeRegister(0x3C, 0x0C);
	// Set MICBIAS voltage 1.25 V
	writeRegister(0x33, 0x40);
	// Select Page 0
	writeRegister(PAGE_SELECT_REGISTER, 0);
	// Power up Left and Right ADC Channels
	writeRegister(0x51, 0xC0);
	// Unmute Left and Right ADC Digital Volume Control.
	writeRegister(0x52, 0x00);
}

static void tlv320aic3204_DeInit(void)
{
	tlv320aic3204_muteControl(1); // mute channels
	tlv320aic3204_LDO_PowerCtrl(0); // LDO power down
	// de-init I2S2 TX interface
	if (HAL_I2S_DeInit(&hi2s2) != HAL_OK)
	{
		 Error_Handler();
	}
	// de-init SPI1 codec control interface
	if (HAL_I2C_DeInit(&hi2c2) != HAL_OK)
    {
	  Error_Handler();
    }
	
	// I2S2 RX DMA interrupt
	HAL_NVIC_DisableIRQ(DMA1_Stream3_IRQn);
	// I2S2 TX DMA interrupt
	HAL_NVIC_DisableIRQ(DMA1_Stream4_IRQn);
}

static void tlv320aic3204_selectOutputs(OutputsType outputs)
{
	if(currentOutputs != outputs)
	{
		// enable outputs mute
		tlv320aic3204_muteControl(1);
		// change current outputs
		currentOutputs = outputs;
		// Select Page 1
		writeRegister(PAGE_SELECT_REGISTER, 0x01);
		switch(outputs)
		{
			case HEADPHONES:
			default:
				// Route Left DAC to HPL
				writeRegister(0x0C, 0x08);
				// Route Right DAC to HPR
				writeRegister(0x0D, 0x08);
				// Unroute Left DAC from LOL
				writeRegister(0x0E, 0x00);
				// Unroute Right DAC from LOR
				writeRegister(0x0F, 0x00);
				// Power up HPL and HPR drivers
				writeRegister(0x09, 0x30);
				break;

			case LOUDSPEAKERS:
				// Route Left DAC to LOL
				writeRegister(0x0E, 0x08);
				// Route Right DAC to LOR
				writeRegister(0x0F, 0x08);
				// Unroute Left DAC from HPL
				writeRegister(0x0C, 0x00);
				// Unroute Left DAC from HPR
				writeRegister(0x0D, 0x00);
				// Power up LOL and LOR drivers
				writeRegister(0x09, 0x0C);
				break;
		}
		// disable outputs mute
		tlv320aic3204_muteControl(0);
		// Set out driver gain -1 dB (for preventing distortion at max volume on loudspeakers)
		tlv320aic3204_setOutDriverGain((outputs == LOUDSPEAKERS) ? (-1) : (0));
	}
}

static void tlv320aic3204_selectInput(InputsType input)
{
	uint8_t leftMic_PGA_VolCtrl = 0, rightMicPGA_VolCtrl = 0;
	if(currentInput != input)
	{
		// change current outputs
		currentInput = input;
		// Select Page 1
		writeRegister(PAGE_SELECT_REGISTER, 0x01);
		// read left and right MicPGA volume control registers
		leftMic_PGA_VolCtrl = readRegister(0x3B);
		rightMicPGA_VolCtrl = readRegister(0x3C);
		// enable inputs mute
		writeRegister(0x3B, leftMic_PGA_VolCtrl|0x80);
		writeRegister(0x3C, rightMicPGA_VolCtrl|0x80);
		
		switch(input)
		{
			case MIC1:
			default:
				// Route IN1L to LEFT_P with 20K input impedance
				writeRegister(0x34, 0x80);
				// Route IN1R to RIGHT_P with input impedance of 20K
				writeRegister(0x37, 0x80);
				break;

			case MIC3:
				// Route IN3L to LEFT_P with 20K input impedance
				writeRegister(0x34, 0x08);
				// Route IN3L to common mode
				writeRegister(0x3A, 0x08);
				// Route IN3R to RIGHT_P with input impedance of 20K
				writeRegister(0x37, 0x08);
				break;
		}
		// disable inputs mute
		writeRegister(0x3B, leftMic_PGA_VolCtrl&0x7F);
		writeRegister(0x3C, rightMicPGA_VolCtrl&0x7F);
	}
}

static void tlv320aic3204_muteControl(uint8_t is_enabled)
{
	uint8_t reg_value = 0;

	// check params
	if(is_enabled > 1) return;
	// Select Page 1
	writeRegister(PAGE_SELECT_REGISTER, 0x01);

	switch(currentOutputs)
	{
		case HEADPHONES:
		default:
			// mute HPL control
			reg_value = readRegister(0x10) & 0xBF; // reset mute bit
			reg_value |= (is_enabled << 6); // set value for mute bit
			writeRegister(0x10, reg_value);
			// mute HPR control
			reg_value = readRegister(0x11) & 0xBF; // reset mute bit
			reg_value |= (is_enabled << 6); // set value for mute bit
			writeRegister(0x11, reg_value);
			break;

		case LOUDSPEAKERS:
			// mute LOL control
			reg_value = readRegister(0x12) & 0xBF; // reset mute bit
			reg_value |= (is_enabled << 6); // set value for mute bit
			writeRegister(0x12, reg_value);
			// mute LOR control
			reg_value = readRegister(0x13) & 0xBF; // reset mute bit
			reg_value |= (is_enabled << 6); // set value for mute bit
			writeRegister(0x13, reg_value);

			if(is_enabled)
			{
				LS_EN_GPIO_Port->ODR &= ~LS_EN_Pin; // disable out amplifier
			}
			else
			{
				LS_EN_GPIO_Port->ODR |= LS_EN_Pin; // enable out amplifier
			}
			break;
	}
}

static void tlv320aic3204_setOutDriverGain(int8_t gain)
{
	uint8_t reg_value = 0;
	// Select Page 1
	writeRegister(PAGE_SELECT_REGISTER, 0x01);

	//check gain limits
	if(gain < -6) gain = -6;
	if(gain > 29) gain = 29;

	switch(currentOutputs)
	{
		case HEADPHONES:
		default:
			// gain HPL control
			reg_value = readRegister(0x10) & 0xC0; // reset gain bits
			reg_value |= (uint8_t)(gain & 0x3F); // set gain value bits
			writeRegister(0x10, reg_value);
			// gain HPR control
			reg_value = readRegister(0x11) & 0xC0; // reset gain bits
			reg_value |= (uint8_t)(gain & 0x3F); // set gain value bits
			writeRegister(0x11, reg_value);
			break;

		case LOUDSPEAKERS:
			// gain LOL control
			reg_value = readRegister(0x12) & 0xC0; //reset gain bits
			reg_value |= (uint8_t)(gain & 0x3F); // set gain value bits
			writeRegister(0x12, reg_value);
			// gain LOR control
			reg_value = readRegister(0x13) & 0xC0; //reset gain bits
			reg_value |= (uint8_t)(gain & 0x3F); // set gain value bits
			writeRegister(0x13, reg_value);
			break;
	}
}

static void tlv320aic3204_setDigitalDACVolume(int8_t volume)
{
	// Select Page 0
	writeRegister(PAGE_SELECT_REGISTER, 0x00);

	//check volume limits
	if(volume > 48) volume = 48;

	// write DAC gain value for both channels
	writeRegister(0x41, volume);
	writeRegister(0x42, volume);
}

static void tlv320aic3204_LDO_PowerCtrl(uint8_t is_enabled)
{
	uint8_t reg_value = 0;

	// check param
	if(is_enabled > 1) return;
	// Select Page 1
	writeRegister(PAGE_SELECT_REGISTER, 0x01);
	// read LDO control register
	reg_value = readRegister(0x02) & 0xFE;
	// write LDO power state
	writeRegister(0x02, reg_value|is_enabled);
}

//static void tlv320aic3204_BeepTest(void)
//{
//	uint32_t sampleLength = 0x3A980; // sample duration 5 sec
//	uint16_t freq = 1000; // freq 1 kHz
//	// Select Page 0
//	writeRegister(PAGE_SELECT_REGISTER, 0x00);
//	// write sample duration
//	writeRegister(0x49, (uint8_t)((sampleLength>>16) & 0xFF));
//	writeRegister(0x4A, (uint8_t)((sampleLength>>8) & 0xFF));
//	writeRegister(0x4B, (uint8_t)(sampleLength & 0xFF));
//	// write sample frequency
//	writeRegister(0x4C, (uint8_t)((freq>>8) & 0xFF));
//	writeRegister(0x4D, (uint8_t)(freq & 0xFF));
//
//	writeRegister(0x4E, (uint8_t)((freq>>8) & 0xFF));
//	writeRegister(0x4F, (uint8_t)(freq & 0xFF));
//
//	// enable beep generator and set volume -6 dB
//	writeRegister(0x47, 0x86);
//}

static uint16_t tlv320aic3204_getOutRemainingDataSize(void)
{
	return (uint16_t)(__HAL_DMA_GET_COUNTER(hi2s2.hdmatx) & 0xFFFF);
}

static uint16_t tlv320aic3204_getInRemainingDataSize()
{
	return (uint16_t)(__HAL_DMA_GET_COUNTER(hi2s2.hdmarx) & 0xFFFF);
}

static void tlv320aic3204_StartDataTransfer(uint16_t* tx_data, uint16_t* rx_data, uint16_t size)
{
	HAL_I2SEx_TransmitReceive_DMA(&hi2s2, tx_data, rx_data, size);
}
