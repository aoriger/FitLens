/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "ssd1306_fonts.h"
#include <string.h>
#include <stdio.h>
#include "ssd1306.h"
#include <stdlib.h>
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
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define GPS_BUFFER_SIZE 128
uint8_t gps_rx_byte;
char gps_buffer[GPS_BUFFER_SIZE];
uint8_t gps_index = 0;

uint8_t hour, minute, second;
uint8_t satellites;
double latitude, longitude;
float speed_knots, speed_mph;
float altitude_m;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Display_Write(uint8_t cmd) {
    HAL_GPIO_WritePin(DISP_DC_PORT, DISP_DC, GPIO_PIN_RESET); // command mode
    HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS, GPIO_PIN_RESET); // select display
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY); // send command
    HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS, GPIO_PIN_SET); // deselect display
}

// convert NMEA (ddmm.mmmm) to lat/lon degrees
double nmea_to_decimal(const char* nmea, char dir) {
	double val = atof(nmea); // string to double
	int degrees = (int)(val / 100);
	double minutes = val - degrees * 100;
	double dec = degrees + minutes / 60.0;
	if(dir == 'S' || dir == 'W') dec = -dec; // convention-may want to keep letters later
	return dec;
}

void parse_GPGGA(char* sentence) {
	char* token;
	int field = 0;
	char lat[16], lon[16], ns, ew, alt[16], sat[4];

	token = strtok(sentence, ",");
	while(token) {
		field++;
		switch(field) {
			case 2: break;
			case 3: strcpy(lat, token); break;
			case 4: ns = token[0]; break;
			case 5: strcpy(lon, token); break;
			case 6: ew = token[0]; break;
			case 7: strcpy(sat, token); break;
			case 10: strcpy(alt, token); break;
		}
		token = strtok(NULL, ",");
	}
	latitude = nmea_to_decimal(lat, ns);
	longitude = nmea_to_decimal(lon, ew);
	satellites = (uint8_t)atoi(sat);
	altitude_m = atof(alt);
}

void parse_GPRMC(char* sentence) {
	char* token;
	int field = 0;
	char time_str[16], speed[16];

	token = strtok(sentence, ",");
	while(token) {
		field++;
		switch(field) {
			case 2: strcpy(time_str, token); break;
			case 8: strcpy(speed, token); break;
		}
		token = strtok(NULL, ",");
	}
	char h[3], m[3], s[3];
	strncpy(h, time_str, 2); h[2] = '\0';
	strncpy(m, time_str+2, 2); m[2] = '\0';
	strncpy(s, time_str+4, 2); s[2] = '\0';
	hour = atoi(h);
	minute = atoi(m);
	second = atoi(s);
	speed_knots = atof(speed);
	speed_mph = speed_knots * 1.150779;
}

// called when UART receive completes in interrupt mode
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART1) {
		if(gps_rx_byte == '\n') {
			gps_buffer[gps_index] = '\0';
			if(strncmp(gps_buffer, "$GPGGA", 6) == 0) parse_GPGGA(gps_buffer);
			else if(strncmp(gps_buffer, "$GPRMC", 6) == 0) parse_GPRMC(gps_buffer);
			gps_index = 0;
		} else if(gps_index < GPS_BUFFER_SIZE - 1) {
			gps_buffer[gps_index++] = gps_rx_byte;
		}
		HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1);
	}
}

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */

  ssd1306_Init();
  // Start UART receive interrupt
  HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 // display time and speed on first two lines
	 char buf[32];
	 sprintf(buf, "%.2f mph", speed_mph);
	 ssd1306_SetCursor(35, 30);
	 ssd1306_WriteString(buf, Font_6x8, White);

	 sprintf(buf, "%02d:%02d:%02d", hour, minute, second);
	 ssd1306_SetCursor(35, 20);
	 ssd1306_WriteString(buf, Font_6x8, White);

	 // display number of satellites and altitude on next two lines
//	 sprintf(buf, "Sat: %d", satellites);
//	 ssd1306_SetCursor(35, 40);
//	 ssd1306_WriteString(buf, Font_6x8, White);

 //	 sprintf(buf, "Alt: %.1f", altitude_m);
 //	 ssd1306_SetCursor(35, 50);
 //	 ssd1306_WriteString(buf, Font_6x8, White);

	 // display coords
	 sprintf(buf, "%.1f|%.1f", latitude, longitude);
	 ssd1306_SetCursor(35, 40);
	 ssd1306_WriteString(buf, Font_6x8, White);

	 // get values from 2 ADC channels
	 HAL_ADC_Start(&hadc);
     HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
     uint32_t adcLight = HAL_ADC_GetValue(&hadc);
     HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
     uint32_t adcTouch = HAL_ADC_GetValue(&hadc);
     HAL_ADC_Stop(&hadc); // might not be needed

     // modify display brightness in three stages based on observed thresholds - tune up later
     if (adcLight > 3500) {
		 Display_Write(0x81); // set contrast/brightness
		 Display_Write(0xFF); // max brightness
     } else if (adcLight > 3000) {
    	 Display_Write(0x81); // set contrast/brightness
    	 Display_Write(0x7F); // medium brightness
     } else {
    	 Display_Write(0x81); // set contrast/brightness
    	 Display_Write(0x00); // lowest brightness
     }

     // display light and touch sensor values for observation
	 sprintf(buf, "%lu %lu", adcLight, adcTouch);
	 ssd1306_SetCursor(35, 50);
	 ssd1306_WriteString(buf, Font_6x8, White);

	 ssd1306_UpdateScreen();
	 HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI1_CS_Pin|SPI1_DC_Pin|SPI1_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI1_CS_Pin SPI1_DC_Pin SPI1_RST_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|SPI1_DC_Pin|SPI1_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
