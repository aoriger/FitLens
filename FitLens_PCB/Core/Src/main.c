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
#include "usb_device.h"
#include "usbd_cdc_if.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "ssd1306_fonts.h"
#include <string.h>
#include <stdio.h>
//#include "ssd1306.h"
#include <stdlib.h>
#include "gps.h"
#include "mx25_flash.h"
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

CRC_HandleTypeDef hcrc;

QSPI_HandleTypeDef hqspi;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;

/* USER CODE BEGIN PV */
uint32_t flash_id = 0;
uint8_t terminal_rx_byte; // buffer for 1 byte to terminal
extern CRC_HandleTypeDef hcrc;
extern uint8_t export_gpx_flag;
uint32_t last_gps_ptr = 0;
uint16_t adc_values[2];
uint16_t adcLight = 0;
uint16_t adcTouch = 0;
uint8_t gps_logging_enabled = 0;
uint32_t last_log_tick = 0;
#define GPS_LOG_INTERVAL_MS 1000
extern uint32_t total_time;
extern uint32_t time_since_update;
extern float distance_traveled;
extern uint32_t last_update;
uint32_t current_flash_address = 0x000000;
//void get_dist_and_time(float lat, float lon);
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
//static int scroll_x = 128; // right edge

int final_time = 0;
float final_dist = 0.0;

extern UART_HandleTypeDef huart2;

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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_ADC1_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void Execute_GPX_Export(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*void Display_Write(uint8_t cmd) {
    HAL_GPIO_WritePin(DISP_DC_PORT, DISP_DC, GPIO_PIN_RESET); // command mode
    HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS, GPIO_PIN_RESET); // select display
    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY); // send command
    HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS, GPIO_PIN_SET); // deselect display
}
*/
void ReadSensors() {
	// need to reconfigure every time to use blocking conversion; no DMA somehow
//	ADC_ChannelConfTypeDef sConfig = {0};
//	sConfig.Channel = ADC_CHANNEL_5;
//	sConfig.Rank = ADC_REGULAR_RANK_1;
//	sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
//	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	adcLight = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

}

/*void Update_Brightness(uint32_t adcLight)
{
    const uint32_t ADC_DARK  = 1100;
    const uint32_t ADC_BRIGHT = 2500;

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
*/
/*
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

		//	char* msg = street[instr_idx];
		//	char* msg = "Mitch Daniels Boulevard";

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
*/
/*
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

//	 if (!arrived_flag) {
//		 sprintf(buf, "%03lu:%.2f", total_time, distance_traveled);
//		 ssd1306_SetCursor(35, 30);
//		 ssd1306_WriteString(buf, Font_6x8, White);
//	 } else {
//		 sprintf(buf, "%d:%.2f", final_time, final_dist);
//		 ssd1306_SetCursor(35, 30);
//		 ssd1306_WriteString(buf, Font_6x8, White);
//	 }

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
*/
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

void Log_Current_GPS_To_Flash(uint32_t address) {
	uint32_t aligned_buf[32] = {0};
	char* gps_log_str = (char*)aligned_buf;
	uint32_t calculated_crc;

	// DI (distance), TT (total_time), and HR (adcLight/HeartRate placeholder)
	int len = snprintf(gps_log_str, 127,
	              "DT:2026-04-16T%02d:%02d:%02dZ,LA:%.6f,LO:%.6f,SP:%.1f,DI:%.1f,TT:%lu,HR:%u",
	              hour, minute, second, latitude, longitude, speed_mph,
	              distance_traveled, total_time, adcLight);

	calculated_crc = HAL_CRC_Calculate(&hcrc, aligned_buf, len);

	if (address % 4096 == 0) {
	        MX25_SectorErase(address);
	}
	MX25_PageProgram(address, (uint8_t*)aligned_buf, 128);
	MX25_PageProgram(address + 128, (uint8_t*)&calculated_crc, 4);

	current_flash_address += 256;
	if (current_flash_address > 0x7FFF00) current_flash_address = 0;
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_ADC1_Init();
  MX_QUADSPI_Init();
  MX_USB_DEVICE_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

  // initialize display
   //ssd1306_Init();

   last_update = HAL_GetTick();

   // initialize GPS/DMA
   HAL_UART_Receive_DMA(&huart4, gps_rx_buf, GPS_RX_BUF_SIZE);
   __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);

   // get route
    done = 1;// 0 to turn on nav

    while(!done) {
  	HAL_UART_Receive(&huart2, (uint8_t*)&nav_c, 1, HAL_MAX_DELAY);

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
    flash_id = MX25_READID_QSPI();
    if (flash_id == 0xC22017) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

        char welcome[] = "\r\n FitLens On\r\nReady for commands [W, G, D]\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)welcome, strlen(welcome), 100);
    }

    // Start listening for commands on the term
    HAL_UART_Receive_IT(&huart2, &terminal_rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t current_gps_ptr = GPS_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart4.hdmarx);
	  gps_process_data(gps_rx_buf, current_gps_ptr);
	    Update_Route();
		//Display_Info();
		ReadSensors(); // get values from 2 ADC channels - light and touch
		//Update_Brightness(adcLight);



		// ------------------- testing

		 // display light and touch sensor values for observation
	  	 char buf[32];
	  	 sprintf(buf, "%d", adcLight);
	  	 //ssd1306_SetCursor(35, 30);
	  	 //ssd1306_WriteString(buf, Font_6x8, White);

	  //	 ssd1306_UpdateScreen();

	  	if (gps_logging_enabled) {
	  	    uint32_t now = HAL_GetTick();

	  	    if ((now - last_log_tick) >= GPS_LOG_INTERVAL_MS &&
	  	        latitude != 0.0 && longitude != 0.0) {
	  	        Log_Current_GPS_To_Flash(current_flash_address);
	  	        last_log_tick = now;
	  	    }
	  	}
	  	if (export_gpx_flag) {
	  	    Execute_GPX_Export();
	  	    export_gpx_flag = 0;
	  	}
		 HAL_Delay(30);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

void Execute_GPX_Export(void) {
	static char xml_buf[256];
	    uint8_t read_data[128];

	    HAL_Delay(100);


	    #define SEND_USB(s, len) { \
	        uint32_t timeout = 100000; \
	        while(CDC_Transmit_FS((uint8_t*)s, len) == USBD_BUSY && timeout > 0) { timeout--; } \
	    }

	    SEND_USB("<?xml version=\"1.1\" encoding=\"UTF-8\"?>\r\n", 40);
	    SEND_USB("<gpx version=\"1.1\" creator=\"FitLens\"><trk><trkseg>\r\n", 52);

	    for (uint32_t addr = 0; addr < current_flash_address; addr += 256) {
	        MX25_ReadData(addr, read_data, 128);
	        if (read_data[0] == 'D') {
	            float l_lat, l_lon, l_spd, l_dist;
	            uint32_t l_tt;
	            unsigned int l_hr;
	            int h, m, s;


	            if (sscanf((char*)read_data, "DT:2026-04-16T%d:%d:%dZ,LA:%f,LO:%f,SP:%f,DI:%f,TT:%lu,HR:%u",
	                       &h, &m, &s, &l_lat, &l_lon, &l_spd, &l_dist, &l_tt, &l_hr) >= 5) {

	                int len = snprintf(xml_buf, sizeof(xml_buf),
	                          "<trkpt lat=\"%.6f\" lon=\"%.6f\"><time>2026-04-16T%02d:%02d:%02dZ</time>"
	                          "<extensions><gpxtpx:hr>%u</gpxtpx:hr></extensions></trkpt>\r\n",
	                          l_lat, l_lon, h, m, s, l_hr);

	                SEND_USB(xml_buf, len);
	                HAL_Delay(10);
	            }
	        }
	    }
	    SEND_USB("</trkseg></trk></gpx>\r\n", 23);
	    #undef SEND_USB
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

  ADC_MultiModeTypeDef multimode = {0};
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

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
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
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hqspi.Init.FlashSize = 22;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        char cmd = (char)terminal_rx_byte;


        if (cmd == 'D' || cmd == 'd') { //download the GPS log
        	uint8_t read_buf[256] = {0};
        	char response[512];

        	MX25_ReadData(0x000000, read_buf, 255);
        	read_buf[255] = '\0';

        	snprintf(response, sizeof(response), "\r\nFitLens Flash Log\r\n%s\r\n", (char*)read_buf);
        	HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 1000);
        }
        else if (cmd == 'G' || cmd == 'g') { // Log live data to flash
        	Log_Current_GPS_To_Flash(current_flash_address);
        }
        else if (cmd == 'W' || cmd == 'w') { // integrity test
            uint8_t test_data[] = "FITLENS_SECURE_BOOT";
            uint8_t verify_buf[20] = {0};
            MX25_SectorErase(0x500000); // Use a far sector to avoid GPS logs
            MX25_PageProgram(0x500000, test_data, 19);
            MX25_ReadData(0x500000, verify_buf, 19);
            if (memcmp(test_data, verify_buf, 19) == 0) {
                HAL_UART_Transmit(&huart2, (uint8_t*)"\r\nMemory Integrity: PASSED\r\n", 28, 100);
            } else {
                HAL_UART_Transmit(&huart2, (uint8_t*)"\r\nMemory Integrity: FAILED\r\n", 28, 100);
            }
        }

        //continues to listen
        HAL_UART_Receive_IT(&huart2, &terminal_rx_byte, 1);
    }
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
