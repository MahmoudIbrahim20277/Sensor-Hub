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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "ssd1306.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//#define UART_DEBUG_ENABLE 0x01
#define DHT11_Pin GPIO_PIN_2
#define DHT11_GPIO_Port GPIOA
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void TerminalNewLine(void);
void ADC_ChangeChannle(uint32_t Ch);
void ReadAdcvalue_Custom(void);
void PotentionMeterHandler(void);
void LM35Handler(void);
void delay_us(uint16_t us);
void DHT11_Start(void);
uint8_t DHT11_Check_Response(void);
uint8_t DHT11_Read_Byte(void);
void DHT11_Process_Data(void);
void send_uart_message(char *message);
void DHT11_handler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*Shared Vars*/
uint32_t ADC_reader = 0.00;
typedef struct
{
	uint32_t integarPart;
	uint32_t decimalPart;
}ADC_Readed_t;
ADC_Readed_t ADC_Converted;
uint8_t Msg[50];

/*For DH11*/
uint8_t T_Byte1, T_Byte2, RH_Byte1, RH_Byte2, CheckSum;

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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  send_uart_message("Initialization complete\r\n");
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#ifdef UART_DEBUG_ENABLE
  HAL_UART_Transmit(&huart1, "Uart Say Hello\r", strlen("Uart Say Hello\r"), 100);
#endif
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  DHT11_handler();
	  PotentionMeterHandler();
	  LM35Handler();
	  TerminalNewLine();

	  //HAL_Delay(2000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void ADC_ChangeChannle(uint32_t Ch)
{
	 ADC_ChannelConfTypeDef sConfig = {0};
	    // Configure the selected ADC channel
	    sConfig.Channel = Ch;
	    sConfig.Rank = ADC_REGULAR_RANK_1;
	    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
	    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
	        // Configuration error
	        Error_Handler();
	    }
	  /** Configure Regular Channel*/
}
void ReadAdcvalue_Custom(void)
{
	//HAL_ADC_ConfigChannel(&hadc1, = ADC_CHANNEL_0 );
	HAL_ADC_Start(&hadc1);
	// Poll for conversion completion
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	ADC_reader = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
#ifdef UART_DEBUG_ENABLE
	sprintf(Msg , "No of Steps = %d\r" , ADC_reader);
	HAL_UART_Transmit(&huart1, (uint8_t *)Msg, sizeof(Msg), HAL_MAX_DELAY);
#endif
}
void PotentionMeterHandler(void)
{
	ADC_ChangeChannle(ADC_CHANNEL_0);
	ReadAdcvalue_Custom();
	uint32_t MILLIVolt = 5000 *  ADC_reader / 4095 ;
	ADC_Converted.integarPart = (MILLIVolt / 1000);
	ADC_Converted.decimalPart = (MILLIVolt % 1000);
	sprintf((char *)Msg , "Value = %d.%d V \r" , ADC_Converted.integarPart , ADC_Converted.decimalPart);
	HAL_UART_Transmit(&huart1, (uint8_t *)Msg, sizeof(Msg), HAL_MAX_DELAY);
}
void LM35Handler(void)
{
	ADC_ChangeChannle(ADC_CHANNEL_1);
	ReadAdcvalue_Custom();
	uint32_t Valuein = (5000 *  ADC_reader / 4095) * 100 ;/* Conveterd to Degrees */
	ADC_Converted.integarPart = (Valuein / 1000);
	ADC_Converted.decimalPart = (Valuein % 1000);
	sprintf((char *)Msg , "LM35 = %d.%d *C \r" , ADC_Converted.integarPart , ADC_Converted.decimalPart);
	HAL_UART_Transmit(&huart1, (uint8_t *)Msg, sizeof(Msg), HAL_MAX_DELAY);

}
void TerminalNewLine(void)
{
	HAL_UART_Transmit(&huart1, "--------------------\r",strlen("--------------------\r"), HAL_MAX_DELAY);
}

void DHT11_handler(void)
{
	HAL_Delay(2000); // Delay 2 seconds between reads
	DHT11_Start();

	if (DHT11_Check_Response())
	{
		T_Byte1 = DHT11_Read_Byte();
		T_Byte2 = DHT11_Read_Byte();
		RH_Byte1 = DHT11_Read_Byte();
		RH_Byte2 = DHT11_Read_Byte();
		CheckSum = DHT11_Read_Byte();

		DHT11_Process_Data();
	}
	else
	{
		send_uart_message("No response from DHT11 sensor\r\n");
	}
	TerminalNewLine();
}

void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

/* Start signal for DHT11 */
void DHT11_Start(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure GPIO pin as output
    GPIO_InitStruct.Pin = DHT11_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

    // Send start signal
    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);
    HAL_Delay(18);  // Keep low for at least 18ms
    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);
    delay_us(30);   // Wait for 20-40us

    // Configure GPIO pin as input
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}
/* Check response from DHT11 */
uint8_t DHT11_Check_Response(void)
{
    delay_us(40); // Wait for 40us
    if (!(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)))
    {
        delay_us(80); // Wait for 80us

        if (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))
        {
            delay_us(40); // Wait for 40us
            return 1;  // Response OK
        }
    }
    return 0;  // No response
}

/* Read one byte from DHT11 */
uint8_t DHT11_Read_Byte(void)
{
    uint8_t i, value = 0;

    for (i = 0; i < 8; i++)
    {
        while (!(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))); // Wait for pin to go high
        delay_us(40); // Wait for 40us

        if (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))
        {
            value |= (1 << (7 - i));  // Read bit
        }

        while (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)); // Wait for pin to go low
    }

    return value;
}

/* Process and display the data */
void DHT11_Process_Data(void)
{
    uint8_t humidity_integer = RH_Byte1;
    uint8_t humidity_decimal = RH_Byte2;
    uint8_t temperature_integer = T_Byte1;
    uint8_t temperature_decimal = T_Byte2;

    if (CheckSum == (RH_Byte1 + RH_Byte2 + T_Byte1 + T_Byte2))
    {
        char msg[50];
        sprintf(msg, "Humidity: %d.%d%%, Temp: %d.%dC\r\n", humidity_integer, humidity_decimal, temperature_integer, temperature_decimal);
        send_uart_message(msg);
    }
    else
    {
        send_uart_message("Checksum error\r\n");
    }
}

/* Send message over UART */
void send_uart_message(char *message)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
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
