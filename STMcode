/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
double pressure = 0;
float elevation = 0;

const uint8_t READ_PRESSURE = 0b10000100; // 0x04 + 0b10000000
const uint8_t READ_TEMP = 0b10000111; // 0x07 + 0b10000000
const uint8_t READ_PRESSURE_AND_TEMP = 0b10000100; // 0x04 + 0b10000000

const uint8_t READ_ENABLES = 0b10011011; // 0x1B + 0b10000000
const uint8_t READ_OSR = 0b10011100; // 0x1C + 0b10000000
const uint8_t READ_STATUS = 0b10000011; // 0x03 + 0b10000000
const uint8_t READ_IF_CONF = 0b10011010; // 0x1A + 0b10000000

const uint8_t WRITE_ENABLES[2] = {0x1B + 0b00000000, 0b00010011}; // {0x1B + 0b00000000, 0b00010011} pressure and temperature Enable
// Forced mode: [5:4]==01 and pressure enabled: bit[0] == 1

const uint8_t WRITE_OSR[2] = {0b00011100, 0b00000000}; // {0x1C + 0b00000000, 0b00000000} osr_t == 000  osr_p == 000
// {0x1C + 0b00000000, 0b00011000} osr_t == 011  osr_p == 000

// parameters for conversion of temperature sensor values to real temperature value
uint16_t NVM_PAR_T1;
uint16_t NVM_PAR_T2;
int8_t NVM_PAR_T3;
double t_lin;

// parameters for conversion of pressure sensor values to real pressure value
int16_t NVM_PAR_P1;
int16_t NVM_PAR_P2;
int8_t NVM_PAR_P3;
int8_t NVM_PAR_P4;
uint16_t NVM_PAR_P5;
uint16_t NVM_PAR_P6;
int8_t NVM_PAR_P7;
int8_t NVM_PAR_P8;
int16_t NVM_PAR_P9;
int8_t NVM_PAR_P10;
int8_t NVM_PAR_P11;

static const uint8_t MICSVZ89_ADDR = 0x70 << 1;
static const uint8_t HIH7000_ADDR = 0x27 << 1;
static const BMP390_I2C_ADDRESS = 0x76 << 1;

// BMP390 Register addresses
#define BMP390_REG_CHIP_ID       0x00
#define BMP390_REG_PRESSURE_DATA 0x04
#define BMP390_REG_CONFIG        0x1F
#define BMP390_REG_PWR_CTRL      0x1B
#define BMP390_CALIB_DATA 		0x31
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void get_NVM_parameters(void){
	// get parameters from (non-volatile memory) for output compensation
	HAL_I2C_Mem_Read(&hi2c1, BMP390_I2C_ADDRESS, 0x31, 1, read_buffer, 22, HAL_MAX_DELAY);
	uint8_t READ_NVM_PAR_addr = 0x31 + 0b10000000;  // base address of NVM_PAR_T1 + read bit

	//Temperature
	uint16_t NVM_PAR_T1 = (uint16_t)(read_buffer[1]<<8) | (uint16_t)read_buffer[0];
	uint16_t NVM_PAR_T2 = (uint16_t)(read_buffer[3]<<8) | (uint16_t)read_buffer[2];
	int8_t NVM_PAR_T3 = (int8_t) read_buffer[4];

	//Pressure
	int16_t NVM_PAR_P1 = (int16_t)(read_buffer[6]<<8) | (int16_t)read_buffer[5];
	int16_t NVM_PAR_P2 = (int16_t)(read_buffer[8]<<8) | (int16_t)read_buffer[7];
	int8_t NVM_PAR_P3 = (int8_t)read_buffer[9];
	int16_t NVM_PAR_P4 = (int8_t)read_buffer[10];
	uint16_t NVM_PAR_P5 = (uint16_t)(read_buffer[12]<<8) | (uint16_t)read_buffer[11];
	uint16_t NVM_PAR_P6 = (uint16_t)(read_buffer[14]<<8) | (uint16_t)read_buffer[13];
	int8_t NVM_PAR_P7 = (int8_t)read_buffer[15];
	int8_t NVM_PAR_P8 = (int8_t)read_buffer[16];
	int16_t NVM_PAR_P9 = (int16_t)(read_buffer[18]<<8) | (int16_t)read_buffer[17];
	int8_t NVM_PAR_P10 = (int8_t)read_buffer[19];
	int8_t NVM_PAR_P11 = (int8_t)read_buffer[20];
}

static float BMP390_compensate_temperature(uint32_t uncomp_temp, struct BMP390_calib_data *calib_data){
	/* Variable to store the compensated temperature */
	float partial_data1;
	float partial_data2;

	float power_val;

	/* 2^(-8) */
	power_val = 0.00390625f;
	float PAR_T1 = (float)NVM_PAR_T1 / power_val;

	/* 2^30 */
	power_val = 1073741824.0f;
	float PAR_T2 = (float)NVM_PAR_T2 / power_val;

	/* 2^48 */
	power_val = 281474976710656.0f;
	float PAR_T3 = (float)NVM_PAR_T3 / power_val;

	partial_data1 = (float)(uncomp_temp - calib_data->PAR_T1);
	partial_data2 = (float)(partial_data1 * calib_data->PAR_T2);

	calib_data->t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data->PAR_T3;

	return calib_data->t_lin;
}

static float BMP390_compensate_pressure(uint32_T uncomp_press, struct BMP390_calib_data *calib_data){
	/* Variable to store the compensated pressure */
	float comp_press;
	float power_val;

	/* Temporary variables used for compensation */
	float partial_data1;
	float partial_data2;
	float partial_data3;
	float partial_data4;

	float partial_out1;
	float partial_out2;

	/* 1 / 2^20 */
	power_val = 1048576.0f;
	double PAR_P1 = (double)(NVM_PAR_P1 - 16384) / power_val;

	/* 1 / 2^29 */
	power_val = 536870912.0f;
	double PAR_P2 = (double)(NVM_PAR_P2 - 16384) / power_val;

	/* 1 / 2^32 */
	power_val = 4294967296.0f;
	double PAR_P3 = (double)NVM_PAR_P3 / power_val;

	/* 1 / 2^37 */
	power_val = 137438953472.0f;
	double PAR_P4 = (double)NVM_PAR_P4 / power_val;

	/* 1 / 2^(-3) */
	power_val = 0.125f;
	double PAR_P5 = (double)NVM_PAR_P5 / power_val;

	/* 1 / 2^6 */
	power_val = 64.0f;
	double PAR_P6 = (double)NVM_PAR_P6 / power_val;

	/* 1 / 2^8 */
	power_val = 256.0f;
	double PAR_P7 = (double)NVM_PAR_P7 / power_val;

	/* 1 / 2^15 */
	power_val = 32768.0f;
	double PAR_P8 = (double)NVM_PAR_P8 / power_val;

	/* 1 / 2^48 */
	power_val = 281474976710656.0f;
	double PAR_P9 = (double)NVM_PAR_P9 / power_val;

	/* 1 / 2^48 */
	power_val = 281474976710656.0f;
	double PAR_P10 = (double)NVM_PAR_P10 / power_val;

	/* 1 / 2^65 */
	power_val = 36893488147419103232.0f;
	double PAR_P11 = (double)NVM_PAR_P11 / power_val;

	/* Calibration data */
	partial_data1 = calib_data->PAR_P6 * calib_data->t_lin;
	partial_data2 = calib_data->PAR_P7 * (calib_data->t_lin * calib_data->t_lin);
	partial_data3 = calib_data->PAR_P8 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
	partial_out1 = calib_data->PAR_P5 + partial_data1 + partial_data2 + partial_data3;

	partial_data1 = calib_data->PAR_P2 * calib_data->t_lin;
	partial_data2 = calib_data->PAR_P3 * (calib_data->t_lin * calib_data->t_lin);
	partial_data3 = calib_data->PAR_P4 * (calib_data->t_lin * calib_data->t_lin * calib_data->t_lin);
	partial_out2 = (float)uncomp_press * (calib_data->PAR_P1 + partial_data1 + partial_data2 + partial_data3);

	partial_data1 = (float)uncomp_press * (float)uncomp_press;
	partial_data2 = calib_data->PAR_P9 + calib_data->PAR_P10 * calib_data->t_lin;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + ((float)uncomp_press * (float)uncomp_press * (float)uncomp_press) * calib_data->PAR_P11;

	comp_press = partial_out1 + partial_out2 + partial_data4;

	return comp_press
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	/* USER CODE BEGIN 1 */
	HAL_StatusTypeDef control;
	uint8_t tbuf[100];
	uint8_t dbuf[50];


	/*Temperature*/
	uint16_t rawtemp = 0;
	float temp = 0;
	unsigned int dectemp = 0;
	unsigned int fractemp = 0;

	/*Humidity*/
	uint16_t rawhumid = 0;
	unsigned int dechum = 0;
	float humid = 0;
	unsigned int frachum = 0;

	/*VOC*/
	float voc = 0;
	unsigned int decvoc = 0;
	uint8_t rawvoc = 0;

	/*CO2*/
	float co2 = 0;
	uint8_t rawco2 = 0;
	unsigned int decco2 = 0;

	/*Light*/
	uint16_t lux = 0;

	/*Pressure*/
	uint32_t raw_pressure = 0;
	float pressure = 0;
	unsigned int decpressure = 0;
	unsigned int fracpressure = 0;

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  get_NVM_parameters();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  // Temperature and humdity
	  	dbuf[0] = 0x00;

	  	control = HAL_I2C_Master_Transmit(&hi2c1, HIH7000_ADDR, dbuf, 1, HAL_MAX_DELAY);
	  	HAL_Delay(250);
	  	if ( control != HAL_OK ) {
	  		strcpy((char*)tbuf, "Error Tx\r\n");
	  	} else {
	  		control = HAL_I2C_Master_Receive(&hi2c1, HIH7000_ADDR, dbuf, 4, HAL_MAX_DELAY);
	  		HAL_Delay(250);
	  		if ( control != HAL_OK ) {
	  			strcpy((char*)tbuf, "Error Rx\r\n");
	  	} else {
	  		//humidity
	  		rawhumid = ((uint16_t)(dbuf[0] & 0b00111111) << 8) | dbuf[1];
	  		humid = ((float)rawhumid / 16382.0) * 100.0;
	  		dechum = (unsigned int) humid;
	  		frachum = (humid - (float)dechum) * 100;

  			//temperature
  			rawtemp = (((uint16_t)dbuf[2] << 6) | (dbuf[3] >> 2));
  			temp = ((float)rawtemp / 16382.0) * 165.0 - 40.0;
  			dectemp = (unsigned int) temp;
  			fractemp = (temp - (float)dectemp) * 100;
  			}
  		}

  		//VOC and CO2
  		dbuf[0] = 0x0C;

  		control = HAL_I2C_Master_Transmit(&hi2c1, MICSVZ89_ADDR, dbuf, 1, HAL_MAX_DELAY);
  		HAL_Delay(250);
  		if ( control != HAL_OK ) {
  			strcpy((char*)tbuf, "Error Tx\r\n");
  		} else {
  			control = HAL_I2C_Master_Receive(&hi2c1, MICSVZ89_ADDR, dbuf, 2, HAL_MAX_DELAY);
  			HAL_Delay(250);
  			if ( control != HAL_OK ) {
  				strcpy((char*)tbuf, "Error Rx\r\n");
  			} else {
  				//voc
  				rawvoc = dbuf[0];
  				voc = ((float)rawvoc-13)*(1000/229);
  				decvoc = (unsigned int) voc;

  				//co2
  				rawco2 = dbuf[1];
  				co2 = ((float)rawco2-13)*(1000/229)+400;
  				decco2 = (unsigned int) co2;
  			}
  		}
  		//light
  		HAL_ADC_Start(&hadc1);
  		HAL_ADC_PollForConversion(&hadc1, 20);
  		lux = HAL_ADC_GetValue(&hadc1);

  		// Request pressure data from BMP390
  		dbuf[0] = BMP390_REG_PRESSURE_DATA;
  		control = HAL_I2C_Master_Transmit(&hi2c1, BMP390_I2C_ADDRESS, dbuf, 6, HAL_MAX_DELAY);
  		HAL_Delay(250);
  		if (control != HAL_OK) {
  			strcpy((char*)tbuf, "Error Tx\r\n");
  		} else {
  			control = HAL_I2C_Master_Receive(&hi2c1, BMP390_I2C_ADDRESS, dbuf, 6, HAL_MAX_DELAY);
  			HAL_Delay(250);
  			if ( control != HAL_OK ) {
  				strcpy((char*)tbuf, "Error Rx\r\n");
  			} else{
  				// Read 6 bytes of data (pressure and temperature)
  				int sensor_pressure = ((read_buffer[3]<<16) | (read_buffer[2]<<8) | read_buffer[1]);
  				int sensor_temperature = ((read_buffer[6]<<16) | (read_buffer[5]<<8) | read_buffer[4]);

  				float temperature = BMP390_compensate_temperature(sensor_temperature, 5);
  				float pressure = BMP390_compensate_pressure(sensor_pressure, 5);

  				decpressure = (unsigned int) pressure;
  				fracpressure = (pressure - (float)decpressure) * 100;
  			 	}
  			 }

  		//sending information
  		sprintf((char*)tbuf, "T:%u.%02u,RH:%u.%02u,CO2:%u,VOC:%u", dectemp, fractemp, dechum, frachum, decco2, decvoc);
  		HAL_UART_Transmit(&huart2, tbuf, strlen((char*)tbuf), HAL_MAX_DELAY);
  		HAL_UART_Transmit(&huart1, tbuf, strlen((char*)tbuf), HAL_MAX_DELAY);

  		sprintf(tbuf, "Light: %hu \r\n", lux);
  		HAL_UART_Transmit(&huart2, tbuf, strlen((char*)tbuf), HAL_MAX_DELAY);

  		sprintf(tbuf, "Pressure: %u.%02u Pa\r\n", decpressure, fracpressure);
  		HAL_UART_Transmit(&huart2, (uint8_t *)tbuf, strlen(tbuf), HAL_MAX_DELAY);
  		HAL_Delay(750);

  		HAL_Delay(1000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
