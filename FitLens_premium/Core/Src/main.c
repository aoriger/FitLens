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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "ssd1306_fonts.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
//#include "ssd1306.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GPS_BUFFER_SIZE 128
uint8_t gps_rx_byte;
char gps_buffer[GPS_BUFFER_SIZE];
uint8_t gps_index = 0;

uint8_t hour, minute, second;
uint8_t satellites;
double latitude, longitude;
float speed_knots, speed_mph;
float altitude_m;


#define MEM_CS_Pin GPIO_PIN_11
#define MEM_CS_GPIO_Port GPIOB
#define SPI1_DC_Pin GPIO_PIN_10
#define SPI1_DC_GPIO_Port GPIOC
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

static uint32_t delay = 250;
IPCC_HandleTypeDef hipcc;

UART_HandleTypeDef hlpuart1;

QSPI_HandleTypeDef hqspi;

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */
uint32_t flash_id = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_IPCC_Init(void);
static void MX_RTC_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_RF_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*void Display_Write(uint8_t cmd) {
    HAL_GPIO_WritePin(DISP_DC_PORT, DISP_DC, GPIO_PIN_RESET); // command mode
    HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS, GPIO_PIN_RESET); // select display
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY); // send command
    HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS, GPIO_PIN_SET); // deselect display
}*/

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
	if(huart->Instance == LPUART1) {
		if(gps_rx_byte == '\n') {
			gps_buffer[gps_index] = '\0';
			if(strncmp(gps_buffer, "$GPGGA", 6) == 0) parse_GPGGA(gps_buffer);
			else if(strncmp(gps_buffer, "$GPRMC", 6) == 0) parse_GPRMC(gps_buffer);
			gps_index = 0;
		} else if(gps_index < GPS_BUFFER_SIZE - 1) {
			gps_buffer[gps_index++] = gps_rx_byte;
		}
		HAL_UART_Receive_IT(&hlpuart1, &gps_rx_byte, 1);
	}
}

uint32_t MX25_READID_QSPI(void){
	QSPI_CommandTypeDef s_command = {0};
	uint8_t id[3];

	s_command.Instruction = 0x9F;
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.AddressMode = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_1_LINE;
	s_command.NbData = 3;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;


	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK)
	{
		HAL_QSPI_Receive(&hqspi, id, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
	}


	return (id[0] << 16) | (id[1] << 8) | id[2];
}

void MX25_WriteEnable(void){
	QSPI_CommandTypeDef s_command = {0};

	s_command.Instruction = 0x06;
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.AddressMode = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_NONE;
	s_command.NbData = 0;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	if (HAL_QSPI_Command(&hqspi, &s_command,HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
		Error_Handler();
	}

}

uint8_t MX25_ReadStatus(void){
	QSPI_CommandTypeDef s_command = {0};
	uint8_t status = 0;

	s_command.Instruction = 0x05;
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.AddressMode = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = QSPI_DATA_1_LINE;
	s_command.NbData = 1;
	s_command.DummyCycles = 0;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK) {
	        HAL_QSPI_Receive(&hqspi, &status, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
	}
	return status;
}

void MX25_WaitForBusy(void) {
    while ((MX25_ReadStatus() & 0x01) == 0x01) {
        HAL_Delay(1);
    }
}

void MX25_PageProgram(uint32_t Address, uint8_t *pData, uint16_t Size){
	QSPI_CommandTypeDef s_command = {0};

	MX25_WriteEnable();
	s_command.Instruction = 0x02;
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.Address = Address;
	s_command.AddressMode = QSPI_ADDRESS_1_LINE;
	s_command.AddressSize = QSPI_ADDRESS_24_BITS;
	s_command.DataMode = QSPI_DATA_1_LINE;
	s_command.NbData = Size; //256 bytes possible
	s_command.DummyCycles = 0;

	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK) {
	    if (HAL_QSPI_Transmit(&hqspi, pData, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
	        Error_Handler();
	    }
	}
	//uses helper to wait for the chip to finish writing
	while ((MX25_ReadStatus() & 0x01) == 0x01);
}

void MX25_SectorErase(uint32_t Address){
	QSPI_CommandTypeDef s_command = {0};

	MX25_WriteEnable();

	s_command.Instruction = 0x20;
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.Address = Address;
	s_command.AddressMode = QSPI_ADDRESS_1_LINE;
	s_command.AddressSize = QSPI_ADDRESS_24_BITS;
	s_command.DataMode = QSPI_DATA_NONE;
	s_command.DummyCycles = 0;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;


	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
	    Error_Handler();
	}

	while ((MX25_ReadStatus() & 0x01) == 0x01);
}

void MX25_ReadData(uint32_t Address, uint8_t *pData, uint16_t Size){
	QSPI_CommandTypeDef s_command = {0};

	s_command.Instruction = 0x03;
	s_command.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	s_command.Address = Address;
	s_command.AddressMode = QSPI_ADDRESS_1_LINE;
	s_command.AddressSize = QSPI_ADDRESS_24_BITS;
	s_command.DataMode = QSPI_DATA_1_LINE;
	s_command.NbData = Size;
	s_command.DummyCycles = 0;
	s_command.DdrMode = QSPI_DDR_MODE_DISABLE;
	s_command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK) {
	    if (HAL_QSPI_Receive(&hqspi, pData, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
	        Error_Handler();
	    }
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
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
  MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_RTC_Init();
  MX_USB_Device_Init();
  MX_QUADSPI_Init();
  MX_RF_Init();
  /* USER CODE BEGIN 2 */


  BSP_LED_Off(LED_GREEN);
  BSP_LED_Off(LED_RED);
  BSP_LED_Off(LED_BLUE);


  BSP_LED_On(LED_BLUE);
  BSP_LED_Off(LED_BLUE);

  flash_id = MX25_READID_QSPI();


  if (flash_id == 0xC22017) {

      BSP_LED_On(LED_GREEN);
  }
  else if (flash_id == 0x000000 || flash_id == 0xFFFFFF) {

      BSP_LED_On(LED_RED);
      BSP_LED_On(LED_BLUE);
  }
  else {

      BSP_LED_On(LED_RED);
  }


    // new test
    //uint8_t tx_buf[] = "FITLENS_SYSTEM_OK";
    //uint8_t rx_buf[20] = {0};

    //erase sector
    //MX25_SectorErase(0x000000);

    //write the string
    //MX25_PageProgram(0x000000, tx_buf, 17);

    // read the string back
    //MX25_ReadData(0x000000, rx_buf, 17);

    // usb test
    /*char usb_msg[64];
    int test_passed = (strcmp((char*)tx_buf, (char*)rx_buf) == 0);

    if (test_passed) {
          sprintf(usb_msg, "Storage & USB work woohoo! ID: 0xC22017\r\n");
      } else {
          sprintf(usb_msg, "Storage Error :( USB Link works.\r\n");
      }

    //7-1-1
    HAL_Delay(3000);
    CDC_Transmit_FS((uint8_t*)usb_msg, strlen(usb_msg));

    if (test_passed) {
          while(1) { BSP_LED_On(LED_GREEN); }
      } else {
          while(1) {
              BSP_LED_On(LED_RED);
              if (rx_buf[0] == 0) BSP_LED_On(LED_BLUE);
          }
      }
     */
    /*if (strcmp((char*)tx_buf, (char*)rx_buf) == 0) {
    if (strcmp("WRONG_STRING", (char*)rx_buf) == 0) {
          // Works
          while(1) {
              BSP_LED_On(LED_GREEN);
              BSP_LED_Off(LED_RED);
              BSP_LED_Off(LED_BLUE);
          }
      } else {
          // does not work
          while(1) {
              BSP_LED_On(LED_RED);
              // If the buffer is empty (0x00) blink blue too
              if (rx_buf[0] == 0) BSP_LED_On(LED_BLUE);
          }
      }
	*/

    // download gpx test
    /*char gpx_data[] =
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<gpx version=\"1.1\" creator=\"FitLens\">\n"
        "<trk><trkseg>\n"
        "<trkpt lat=\"40.4237\" lon=\"-86.9138\"><ele>188.0</ele></trkpt>\n"
        "</trkseg></trk></gpx>\n";
    MX25_SectorErase(0x000000);
    MX25_PageProgram(0x000000, (uint8_t*)gpx_data, strlen(gpx_data));

    //uint32_t flash_id = MX25_READID_QSPI();

    //notifies user over USB
    HAL_Delay(3000);
    char prompt[128];
    sprintf(prompt, "\r\nFitLens\r\n" "Flash ID: 0x%06lX\r\n" "Commands: [D]ownload, [E]rase, [L]ED Toggle\r\n", flash_id);
    CDC_Transmit_FS((uint8_t*)prompt, strlen(prompt));

    while(1) {
            HAL_Delay(100);
    }
	*/
  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init();

  /* Initialize leds */
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_SW1, BUTTON_MODE_EXTI);
  BSP_PB_Init(BUTTON_SW2, BUTTON_MODE_EXTI);
  BSP_PB_Init(BUTTON_SW3, BUTTON_MODE_EXTI);

  /* USER CODE BEGIN BSP */

  /* -- Sample board code to send message over COM1 port ---- */
  printf("Welcome to STM32 world !\n\r");

  /* -- Sample board code to switch on leds ---- */
  BSP_LED_On(LED_BLUE);
  BSP_LED_On(LED_GREEN);
  BSP_LED_On(LED_RED);

  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MX_APPE_Process();

//    /* -- Sample board code for User push-button in interrupt mode ---- */
//    BSP_LED_Toggle(LED_BLUE);
//    HAL_Delay(delay);
//
//    BSP_LED_Toggle(LED_GREEN);
//    HAL_Delay(delay);
//
//    BSP_LED_Toggle(LED_RED);
//    HAL_Delay(delay);
//
//    ssd1306_Fill(White);
//    ssd1306_UpdateScreen();
//    HAL_Delay(1000);
//    ssd1306_Fill(Black);
//    ssd1306_UpdateScreen();

//    // display time and speed on first two lines
//	 char buf[32];
//	 sprintf(buf, "%.2f mph", speed_mph);
//	 ssd1306_SetCursor(35, 30);
//	 ssd1306_WriteString(buf, Font_6x8, White);
//
//	 sprintf(buf, "%02d:%02d:%02d", hour, minute, second);
//	 ssd1306_SetCursor(35, 20);
//	 ssd1306_WriteString(buf, Font_6x8, White);
//
//	 // display number of satellites and altitude on next two lines
////	 sprintf(buf, "Sat: %d", satellites);
////	 ssd1306_SetCursor(35, 40);
////	 ssd1306_WriteString(buf, Font_6x8, White);
//
// //	 sprintf(buf, "Alt: %.1f", altitude_m);
// //	 ssd1306_SetCursor(35, 50);
// //	 ssd1306_WriteString(buf, Font_6x8, White);
//
//	 // display coords
//	 sprintf(buf, "%.1f|%.1f", latitude, longitude);
//	 ssd1306_SetCursor(35, 40);
//	 ssd1306_WriteString(buf, Font_6x8, White);
//
//	 // get values from 2 ADC channels
////	 HAL_ADC_Start(&hadc1);
////	 HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
////	 uint32_t adcLight = HAL_ADC_GetValue(&hadc1);
////	 HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
////	 uint32_t adcTouch = HAL_ADC_GetValue(&hadc1);
////	 HAL_ADC_Stop(&hadc1); // might not be needed
////
////	 // modify display brightness in three stages based on observed thresholds - tune up later
////	 if (adcLight > 3500) {
////		 Display_Write(0x81); // set contrast/brightness
////		 Display_Write(0xFF); // max brightness
////	 } else if (adcLight > 3000) {
////		 Display_Write(0x81); // set contrast/brightness
////		 Display_Write(0x7F); // medium brightness
////	 } else {
////		 Display_Write(0x81); // set contrast/brightness
////		 Display_Write(0x00); // lowest brightness
////	 }
////
////	 // display light and touch sensor values for observation
////	 sprintf(buf, "%lu %lu", adcLight, adcTouch);
////	 ssd1306_SetCursor(35, 50);
////	 ssd1306_WriteString(buf, Font_6x8, White);
//
//	 ssd1306_UpdateScreen();
//	 HAL_Delay(1000);
    /* USER CODE END WHILE */
    MX_APPE_Process();

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

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV1);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_MSI);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */

  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */

  /* USER CODE END IPCC_Init 2 */

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
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  * @brief RF Initialization Function
  * @param None
  * @retval None
  */
static void MX_RF_Init(void)
{

  /* USER CODE BEGIN RF_Init 0 */

  /* USER CODE END RF_Init 0 */

  /* USER CODE BEGIN RF_Init 1 */

  /* USER CODE END RF_Init 1 */
  /* USER CODE BEGIN RF_Init 2 */

  /* USER CODE END RF_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = CFG_RTC_ASYNCH_PRESCALER;
  hrtc.Init.SynchPrediv = CFG_RTC_SYNCH_PRESCALER;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

hw_status_t HW_UART_Transmit_DMA(hw_uart_id_t hw_uart_id, uint8_t *p_data, uint16_t size, void (*Callback)(void))
{
    /* 1. Send the data using the blocking HAL function (since we know we are using USART1) */
    HAL_UART_Transmit(&hlpuart1, p_data, size, 1000);

    /* 2. Important: The Trace system expects a callback when "DMA" finishes.
       Since we did it blocking, we must call it now so the trace doesn't hang. */
    if (Callback != NULL)
    {
        Callback();
    }

    /* 3. Return 0 (hw_status_ok) */
    return (hw_status_t)0;
}
  #ifdef __GNUC__
  int __io_putchar(int ch)
  #else
  int fputc(int ch, FILE *f)
  #endif
  {
      /* Use huart1 since that is what you initialized in main() */
      HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
      return ch;
  }

//  int _write(int file, char *ptr, int len)
//  {
//      int i;
//      for (i = 0; i < len; i++)
//      {
//          __io_putchar(*ptr++);
//      }
//      return len;
//  }
/* USER CODE END 4 */

/**
  * @brief EXTI line detection callback.
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin)
  {
    case BUTTON_SW1_PIN:
      /* Change the period to 100 ms */
      delay = 100;
      break;
    case BUTTON_SW2_PIN:
      /* Change the period to 500 ms */
      delay = 500;
      break;
    case BUTTON_SW3_PIN:
      /* Change the period to 1000 ms */
      delay = 1000;
      break;
    default:
      break;
  }
}

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
