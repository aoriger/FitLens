/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "gps.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t adcLight = 0;

extern uint32_t total_time;
extern uint32_t time_since_update;
extern float distance_traveled;
extern uint32_t last_update;
extern float distance_m(float lat1, float lon1,
        float lat2, float lon2);

// nav vars
char nav_rx; // nav byte
char nav_buffer[128];
int nav_idx = 0;
char nav_c;
int done;
int instr_idx = 0;
float dist_to_wp;

#define MAX_WAYPOINTS 200
#define MAX_STREET_LEN 32
#define MAX_DIR_LEN 16
#define MAX_DIST_LEN 16

float lat[MAX_WAYPOINTS];
float lon[MAX_WAYPOINTS];
char direction[MAX_WAYPOINTS][MAX_DIR_LEN];
char street[MAX_WAYPOINTS][MAX_STREET_LEN];
char distance_to_turn[MAX_WAYPOINTS][MAX_DIST_LEN];

int waypoint_count = 0;
int arrived_flag = 0;
int first_run = 1;
static int scroll_x = 128; // right edge

int final_time = 0;
float final_dist = 0.0;

//extern UART_HandleTypeDef hlpuart1; // might not need

uint8_t rx_byte;

// 8x8 bitmap for right arrow
uint8_t arrow_right[8] = {
    0b00010000,
    0b00011000,
    0b00001100,
    0b11111110,
    0b11111110,
    0b00001100,
    0b00011000,
    0b00010000
};

uint8_t arrow_left[8] = {
    0b00001000,
    0b00011000,
    0b00110000,
    0b01111111,
    0b01111111,
    0b00110000,
    0b00011000,
    0b00001000
};

// 8x8 bitmap for straight/up arrow
uint8_t arrow_up[8] = {
    0b00011000,
    0b00111100,
    0b01111110,
    0b00011000,
    0b00011000,
    0b00011000,
    0b00011000,
    0b00000000
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1; // might not need?
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_lpuart_rx;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi2_tx;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

void Display_Write(uint8_t cmd) {
    HAL_GPIO_WritePin(DISP_DC_PORT, DISP_DC, GPIO_PIN_RESET); // command mode
    HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS, GPIO_PIN_RESET); // select display
    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY); // send command
    HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS, GPIO_PIN_SET); // deselect display
}

void ReadSensors() {
	// need to reconfigure every time to use blocking conversion; no DMA somehow
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	adcLight = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
}

void Update_Brightness(uint32_t adcLight)
{
    const uint32_t ADC_DARK  = 1300;
    const uint32_t ADC_BRIGHT = 1400;

    const uint8_t BRIGHTNESS_MIN = 0x00;  // calibrate
    const uint8_t BRIGHTNESS_MAX = 0xFF;

    uint8_t brightness;

    if (adcLight <= ADC_DARK)
    {
    	brightness = BRIGHTNESS_MIN;
    }
    else if (adcLight >= ADC_BRIGHT)
    {
        brightness = BRIGHTNESS_MAX;
    }
    else
    {
        brightness = BRIGHTNESS_MIN + (uint8_t)((adcLight - ADC_DARK) * (BRIGHTNESS_MAX - BRIGHTNESS_MIN) /
                (ADC_BRIGHT - ADC_DARK)); // could optimize
    }

    Display_Write(0x81); // update brightness
    Display_Write(brightness); // update to level
}

void Display_Nav() {

	ssd1306_FillRectangle(0, 50, 127, 58, Black);

//		 find substrings and print matching arrow
	    if (strstr(direction[instr_idx], "right"))
	    	ssd1306_DrawBitmap(35, 50, arrow_right, 8, 8, White);
	    else if (strstr(direction[instr_idx], "left"))
	    	ssd1306_DrawBitmap(35, 50, arrow_left, 8, 8, White);
	    else if (strstr(direction[instr_idx], "straight") || strstr(direction[instr_idx], "continue"))
	    	ssd1306_DrawBitmap(35, 50, arrow_up, 8, 8, White);
//	     else?
	    ssd1306_DrawBitmap(35, 50, arrow_right, 8, 8, White); // get rid of this

	int char_width = 6;
	int screen_width = 128;
	int left_margin = 50;
	int gap = 20;

	char msg_buffer[64];
	char dist_buff[32];
	if (latitude == 0) {
		snprintf(dist_buff, sizeof(dist_buff), "?");
	} else {
		snprintf(dist_buff, sizeof(dist_buff), "%.0fm", dist_to_wp);
	}
	snprintf(msg_buffer, sizeof(msg_buffer), "%s %s", dist_buff, street[instr_idx]);
	char* msg = msg_buffer;

	if (!arrived_flag) {
//			char* msg = "Mitch Daniels Boulevard";

			int msg_len = strlen(msg);
			int text_width = msg_len * char_width;
			int cycle_width = text_width + gap;

			for (int copy = 0; copy < 2; copy++) {
				int start_pos = scroll_x + (copy * cycle_width);

				for (int i = 0; i < msg_len; i++) {
					int char_x = start_pos + (i * char_width);
					if (char_x >= left_margin && char_x <= (screen_width - char_width)) {
						ssd1306_SetCursor(char_x, 50);
						ssd1306_WriteChar(msg[i], Font_6x8, White);
					}
				}
			}

			scroll_x--;

			if (scroll_x <= (left_margin - cycle_width)) {
				scroll_x = left_margin;
			}

	} else {
		ssd1306_SetCursor(35, 50);
		ssd1306_WriteString("Arrived", Font_6x8, White);
		if (final_time == 0) {
			// total_time and distance_traveled lowkey keep incrementing -- stop them
			final_time = total_time;
			final_dist = distance_traveled;
		}
	}

}

void Display_Info() {

	if ((latitude == 0.0)) {
		// this is kind of jank and would need to change if display fields are edited - maybe make flags
		// also add clearing the line before writing dashed lines to completely overwrite text
		 char buf[32];
		 sprintf(buf, "----------------");
		 ssd1306_SetCursor(35, 40);
		 ssd1306_WriteString(buf, Font_6x8, White);
	} else {
		 // display coords on line 3
		 char buf[32];
		 sprintf(buf, "%.1f|%.1f ", latitude, longitude);
		 ssd1306_SetCursor(35, 40);
		 ssd1306_WriteString(buf, Font_6x8, White);
	}

    // display time and speed on lines 1/2
	 char buf[32];
//	 sprintf(buf, "%02d:%02d:%02d", hour, minute, second);
	 sprintf(buf, "%02d:%02d", hour, minute);
	 ssd1306_SetCursor(35, 20);
	 ssd1306_WriteString(buf, Font_6x8, White);

//	 sprintf(buf, "%.2f mph", speed_mph);
//	 ssd1306_SetCursor(35, 30);
//	 ssd1306_WriteString(buf, Font_6x8, White);

	 if (!arrived_flag) {
		 sprintf(buf, "%03lu:%.2f", total_time, distance_traveled);
		 ssd1306_SetCursor(35, 30);
		 ssd1306_WriteString(buf, Font_6x8, White);
	 } else {
		 sprintf(buf, "%03d:%.2f", final_time, final_dist);
		 ssd1306_SetCursor(35, 30);
		 ssd1306_WriteString(buf, Font_6x8, White);
	 }

	 // display number of satellites and altitude on lines 3/4
//	 sprintf(buf, "Sat: %d", satellites);
//	 ssd1306_SetCursor(35, 40);
//	 ssd1306_WriteString(buf, Font_6x8, White)
 //	 sprintf(buf, "Alt: %.1f", altitude_m);
 //	 ssd1306_SetCursor(35, 50);
 //	 ssd1306_WriteString(buf, Font_6x8, White);

	 Display_Nav(); // row 4 (50)

	 ssd1306_UpdateScreen();

}

void process_instruction(char *line)
{
    if (strcmp(line, "<END>") == 0)
    {
        return;
    }

    if (waypoint_count >= MAX_WAYPOINTS)
        return;

    char *token;
    float temp_lat, temp_lon;
    char temp_dir[MAX_DIR_LEN];
    char temp_street[MAX_STREET_LEN];
    char temp_dist[MAX_DIST_LEN];

    // split string into fields
    token = strtok(line, ",");
    temp_lat = atof(token);

    token = strtok(NULL, ",");
    temp_lon = atof(token);

    token = strtok(NULL, ",");
    strncpy(temp_dir, token, MAX_DIR_LEN-1);
    temp_dir[MAX_DIR_LEN-1] = '\0';

    token = strtok(NULL, ",");
    strncpy(temp_dist, token, MAX_DIST_LEN-1);
    temp_dist[MAX_DIST_LEN-1] = '\0';

    token = strtok(NULL, ",");
    strncpy(temp_street, token, MAX_STREET_LEN-1);
    temp_street[MAX_STREET_LEN-1] = '\0';

    // Store in arrays
    lat[waypoint_count] = temp_lat;
    lon[waypoint_count] = temp_lon;
    strncpy(direction[waypoint_count], temp_dir, MAX_DIR_LEN);
    strncpy(distance_to_turn[waypoint_count], temp_dist, MAX_DIST_LEN);
    strncpy(street[waypoint_count], temp_street, MAX_STREET_LEN);

    waypoint_count++;
}

void Update_Route() {
	float threshold = 28.0; // vary as needed based on travel speed - maybe make global

	for (int i = 0; i < waypoint_count; i++) {
		dist_to_wp = distance_m(latitude, longitude, lat[i], lon[i]);
		if ((dist_to_wp <= threshold) && (latitude != 0)) {
			if (i+1 < waypoint_count) {
				instr_idx = i + 1;
				dist_to_wp = distance_m(latitude, longitude, lat[i+1], lon[i+1]);
			} else {
				arrived_flag = 1;
			}
			break;
		} else {
			dist_to_wp = distance_m(latitude, longitude, lat[instr_idx], lon[instr_idx]);
		}
	}
}
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USB_PCD_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // initialize display
  ssd1306_Init();

  last_update = HAL_GetTick();

//  // initialize GPS/DMA
  HAL_UART_Receive_DMA(&hlpuart1, gps_rx_buf, GPS_RX_BUF_SIZE);
  __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_IDLE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // get route
   done = 0;// 0 to turn on nav

   while(!done) {
 	HAL_UART_Receive(&hlpuart1, (uint8_t*)&nav_c, 1, HAL_MAX_DELAY);

 	  if (nav_c == '\n')
 	  {
 		  nav_buffer[nav_idx] = '\0';
 		  process_instruction(nav_buffer);
 		  nav_idx = 0;
 		  memset(nav_buffer, 0, sizeof(nav_buffer));
 	  }
 	  else
 	  {
 		  if(nav_idx < sizeof(nav_buffer)-1)
 			  nav_buffer[nav_idx++] = nav_c;
 	  }

 	  if (strcmp(nav_buffer, "<END>") == 0)
 	  {
 	   done = 1;
 	  }
   }

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  Update_Route();
	  	Display_Info();
	  	ReadSensors(); // just light sensor now
//	  	Update_Brightness(adcLight);

	  	Display_Write(0x81); // update brightness
	  	Display_Write(0xFF); // update to level

	  	// ------------------- testing

	  	 // display light and touch sensor values for observation
//	  	 char buf[32];
//	  	 sprintf(buf, "%d", adcLight);
//	  	 ssd1306_SetCursor(35, 30);
//	  	 ssd1306_WriteString(buf, Font_6x8, White);
//
	  	 ssd1306_UpdateScreen();
	  	 HAL_Delay(30);

	  	// blink LED
//	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//	  HAL_Delay(500);
//	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//	  HAL_Delay(500);


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
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
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA2_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Display_CS_Pin|Display_DC_Pin|DATA_READY_Pin|NRF_READY_Pin
                          |STM_READY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Display_RST_GPIO_Port, Display_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|SPI3_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Display_CS_Pin Display_DC_Pin DATA_READY_Pin NRF_READY_Pin
                           STM_READY_Pin */
  GPIO_InitStruct.Pin = Display_CS_Pin|Display_DC_Pin|DATA_READY_Pin|NRF_READY_Pin
                          |STM_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Display_RST_Pin */
  GPIO_InitStruct.Pin = Display_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Display_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin SPI3_NSS_Pin */
  GPIO_InitStruct.Pin = LED_Pin|SPI3_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
