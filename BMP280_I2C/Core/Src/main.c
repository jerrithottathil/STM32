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
#include "bmp280.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>   // for rand()

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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;


/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
BMP280_HandleTypedef bmp280;

float pressure, temperature, altitude;

uint16_t size;
uint8_t Data[256];

RTC_TimeTypeDef sTime;
RTC_DateTypeDef DateToUpdate;

RTC_TimeTypeDef gTime;
RTC_DateTypeDef gDate;

uint8_t time[20];
uint8_t date[20];



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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
//  bmp280_init_default_params(&bmp280.params);
//  	bmp280.addr = BMP280_I2C_ADDRESS_0;
//  	bmp280.i2c = &hi2c1;
//
//  	while (!bmp280_init(&bmp280, &bmp280.params)) {
//  		size = sprintf((char *)Data, "BMP280 initialization failed\n");
//  		HAL_UART_Transmit(&huart2, Data, size, 1000);
//  		HAL_Delay(2000);
//  	}
//  	bool bme280p = bmp280.id == BME280_CHIP_ID;
//  	size = sprintf((char *)Data, "BMP280: found %s\n", bme280p ? "BME280" : "BMP280");
//  	HAL_UART_Transmit(&huart2, Data, size, 1000);



  	//set_date(24, 8, 11, 7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 /*Code for taking from bmp280 sensors and rtc*/
//	  HAL_Delay(100);
//	  /* Get the RTC current Time */
//	  // Calculate altitude using barometric formula
//
//
//
//	    	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
//	    	  /* Get the RTC current Date */
//	    	  HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD);
//
//
//	  		while (!bmp280_read_float(&bmp280, &temperature, &pressure, &altitude)) {
//	  			size = sprintf((char *)Data,
//	  					"Temperature/pressure reading failed\n");
//	  			HAL_UART_Transmit(&huart2, Data, size, 1000);
//	  			HAL_Delay(2000);
//	  		}
//
//	  		 altitude = 44330.0 * (1.0 - pow((pressure / 101325.0), 0.1903));
//	  		size = sprintf((char *)Data,
//	  		  "{ \"pressure\": %.2f, \"temperature\": %.2f, \"altitude\": %.2f, \"time\": \"%02d:%02d:%02d\", \"date\": \"%04d-%02d-%02d\" }\n",
//	  		  pressure, temperature, altitude,
//	  		  sTime.Hours, sTime.Minutes, sTime.Seconds,
//	  		  2000 + DateToUpdate.Year, DateToUpdate.Month, DateToUpdate.Date);
//	  		HAL_UART_Transmit(&huart2, Data, size, 1000);
//
//	  		HAL_UART_Transmit(&huart2, Data, size, 1000);
//
//	  		HAL_Delay(1000);

	  /*Code for taking random values*/

	  HAL_Delay(100);

	      /* Get RTC current time and date */
	      HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
	      HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD);

	      /* Generate random sensor values */
	      float temperature = 20.0f + (rand() % 1500) / 100.0f; // 20.0 – 35.0 °C
	      float dissolvedO2 = 5.0f  + (rand() % 300) / 100.0f;  // 5.0 – 8.0 mg/L
	      float salinity    = 30.0f + (rand() % 1000) / 100.0f; // 30.0 – 40.0 PSU
	      float pH          = 6.5f  + (rand() % 200) / 100.0f;  // 6.5 – 8.5

	      /* Format JSON packet */
	      size = sprintf((char *)Data,
	         "{ \"Timestamp\": \"%04d-%02d-%02dT%02d:%02d:%02dZ\", "
	         "\"Temperature\": %.2f, "
	         "\"Dissolved O2\": %.2f, "
	         "\"Salinity\": %.2f, "
	         "\"pH\": %.2f }\n",
	         2000 + DateToUpdate.Year, DateToUpdate.Month, DateToUpdate.Date,
	         sTime.Hours, sTime.Minutes, sTime.Seconds,
	         temperature, dissolvedO2, salinity, pH);

	      HAL_UART_Transmit(&huart3, Data, size, 1000);

	      HAL_Delay(1000);

//	  HAL_Delay(100);
//
//	  /* Get RTC current time and date */
//	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
//	  HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD);
//
//	  /* --- Generate fake sensor values --- */
//	  float temperature = 20.0f + (rand() % 1500) / 100.0f; // 20.0 – 35.0 °C
//	  float salinity    = 30.0f + (rand() % 1000) / 100.0f; // 30.0 – 40.0 PSU
//	  float battery     = 3.7f;  // Example fixed value
//
//	  /* Packet ID counter */
//	  static uint32_t packet_id = 0;
//	  packet_id++;
//
//	  /* CRC placeholder (implement real CRC if needed) */
//	  uint16_t crc = 9399;
//
//	  /* Format JSON packet */
//	  size = sprintf((char *)Data,
//	     "{ \"header\": \"DATA\", "
//	     "\"device_id\": \"NODE_001\", "
//	     "\"rtc\": \"%04d/%d/%d %02d:%02d:%02d\", "
//	     "\"packet_id\": %lu, "
//	     "\"battery\": %.2f, "
//	     "\"TDS\": %d, "
//	     "\"EC\": %d, "
//	     "\"Salinity\": %.2f, "
//	     "\"Temp\": %.2f, "
//	     "\"crc\": \"%u\" }\n",
//	     2000 + DateToUpdate.Year, DateToUpdate.Month, DateToUpdate.Date,
//	     sTime.Hours, sTime.Minutes, sTime.Seconds,
//	     packet_id, battery, 0, 0, salinity, temperature, crc);
//
//	  HAL_UART_Transmit(&huart3, Data, size, 1000);
//
//  }
  /* USER CODE END 3 */
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
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
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
#define RTC_BKP_DEFINE_CODE 0x32F2
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
//
  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */
//
  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
//
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != RTC_BKP_DEFINE_CODE)
      {
        // RTC not initialized before: Set default time and date
  	  sTime.Hours = 0x03;
  	    sTime.Minutes = 0x0;
  	    sTime.Seconds = 0x0;

  	    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  	    {
  	      Error_Handler();
  	    }
  	  DateToUpdate.WeekDay = RTC_WEEKDAY_THURSDAY;
  	    DateToUpdate.Month = RTC_MONTH_JULY;
  	    DateToUpdate.Date = 0x31;
  	    DateToUpdate.Year = 0x0;

  	    if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  	    {
  	      Error_Handler();
  	    }



        // Write magic code to backup register
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, RTC_BKP_DEFINE_CODE);
      }

//
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
//  sTime.Hours = 0x0;
//  sTime.Minutes = 0x0;
//  sTime.Seconds = 0x0;
//
//  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  DateToUpdate.WeekDay = RTC_WEEKDAY_THURSDAY;
//    DateToUpdate.Month = RTC_MONTH_JULY;
//    DateToUpdate.Date = 0x31;
//    DateToUpdate.Year = 0x0;
//
//  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN RTC_Init 2 */
//
  /* USER CODE END RTC_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
