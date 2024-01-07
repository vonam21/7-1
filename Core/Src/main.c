/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c-lcd.h"
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
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define CYCLE_MAX 50
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
uint32_t count,cycles,count_cycles[100],arr[100];
uint8_t data[4];
uint8_t arr_low[50];
uint8_t arr_high[50];
uint8_t arr_endline[50] = "\n\n";
uint32_t count_cycles_state(bool n) {
    count =0;
    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == n) {
        count++;
	if(count > 50) {
		return count;
	}
    }
    return count;
}
int Read_DHT11(uint32_t* arr,uint8_t* data) {

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT ;
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_Delay(1);


  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP  ;
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

  HAL_Delay(20);

  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT ;
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  __disable_irq();
  for(int i=0;i< 160;i++) {
	  __NOP();
  }
  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0);
  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 1);

  for(int i=0; i< 100 ; i+=2) {
	  count_cycles[i] = count_cycles_state(0);
	  count_cycles[i+1] = count_cycles_state(1);
	  arr[i] = count_cycles[i];
	  arr[i+1] = count_cycles[i+1];
  }
  __enable_irq();
  data[1] = 0x00;
  data[2] = 0x00;
  data[3] = 0x00;
  data[4] = 0x00;
  for (int i = 0; i < 40; ++i) {
	  uint32_t lowCycles = arr[2 * i];
	  uint32_t highCycles = arr[2 * i + 1];
	  if ((lowCycles >= CYCLE_MAX) || (highCycles >= CYCLE_MAX)) {
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  }
	  data[i / 8] <<= 1;
	  if (highCycles > lowCycles) {
		  data[i / 8] |= 1;
	  }

  }
  uint8_t check_sum = ( data[0]+ data[1]+ data[2]+ data[3] ) & 0xFF;
  if(check_sum == data[4]) {

  }else {
	  unsigned char err[] = "data loi\r\n";
	  HAL_UART_Transmit(&huart1, err, strlen((char *)err), 10);
	  data[1] = 0x00;
	  data[2] = 0x00;
	  data[3] = 0x00;
	  data[4] = 0x00;
	  return -1;
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
	uint8_t buffer[20]= "gia tri ADC la:";
	uint16_t var_adc =0,var_adc_2=0;
	char buffer_adc[50],buffer_adc_2[50];
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc1);
  HAL_Delay(50);
  HAL_ADC_Start(&hadc2);
  HAL_Delay(50);
  lcd_init();
  lcd_clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	  HAL_Delay(10000);
	  	  HAL_UART_Transmit(&huart1, buffer,strlen((char*)buffer),10);
	  	  HAL_Delay(5000);
	  	  Read_DHT11(arr,data); // đ ?c dht11
	  	  int a =data[2];
	  	  int b =data[3];
	  	  int c =data[0];
	  	  int d =data[1];
	  	  char tx_humi[100];
	  	  char string_lcd_line1[50],string_lcd_line2[50];
	  	  char tx[100];
	  	  char space[]  = " ";
	  	  char end[] = "\r\n\r\n\r\n\r\n\r\n";
	  	  char total[300];


	  	  sprintf(tx_humi, "%d.%d", a, b);
	  	  sprintf(tx, "%d.%d", c, d);
	  	  var_adc = HAL_ADC_GetValue(&hadc1);   // đ ?c adc quang trở
	  	  var_adc_2 = HAL_ADC_GetValue(&hadc2);
	  	  sprintf(buffer_adc, "%d", var_adc);
	  	  sprintf(buffer_adc_2, "%d", var_adc_2);
	  	for(int i=0;i<300;i++) {
	  		  			  total[i] =0;
	  		  		  }
	  	  strcat(total,tx_humi);
	  	  strcat(total,space);
	  	  strcat(total,tx);
	  	  strcat(total,space);
	  	  strcat(total,buffer_adc);
	  	  strcat(total,space);
	  	  strcat(total,buffer_adc_2);
	  	  strcat(total,end);
	  //	  HAL_UART_Transmit(&huart1, total, strlen(total), 10);


	  	  HAL_Delay(3000);

	  	  unsigned char data_tat[] = "AT+CGEREP=0\r\n";
	  	  HAL_UART_Transmit(&huart1, data_tat, strlen((char *)data_tat), 10);
	  	  HAL_Delay(1000);

	  	  unsigned char data1[] = "AT+CMQTTSTART\r\n";
	  	  HAL_UART_Transmit(&huart1, data1, strlen((char *)data1), 10);
	  	  HAL_Delay(1000);

	  	  unsigned char data2[] = "AT+CMQTTACCQ=0,\"clientID\"\r\n";
	  	  HAL_UART_Transmit(&huart1, data2, strlen((char *)data2), 10);
	  	  HAL_Delay(1000);


	  	  while(1) {
	  		  Read_DHT11(arr,data);
	  		  a =data[2];
	  		  b =data[3];
	  		  c =data[0];
	  		  d =data[1];
	  		  unsigned char err1[] = "data loi 1\r\n";
	  		  HAL_UART_Transmit(&huart1, err1, strlen((char *)err1), 10);
	  		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  		  lcd_clear();
	  		  sprintf(string_lcd_line1, "nhietdo: %d.%doC", a,b);
			  sprintf(string_lcd_line2, "doam: %d.%d", c,d);
			  strcat(string_lcd_line2,"%");
			  lcd_put_cur(0,0);
			  lcd_send_string(string_lcd_line1);
			  lcd_put_cur(1,0);
			  lcd_send_string(string_lcd_line2);
	  		  sprintf(tx_humi, "%d.%d", a, b);
	  		  sprintf(tx, "%d.%d", c, d);
	  		  HAL_ADC_Start(&hadc1);
	  		  HAL_Delay(50);
	  		  var_adc = HAL_ADC_GetValue(&hadc1);   // đ ?c adc quang trở
	  		  HAL_ADC_Start(&hadc2);
	  		  HAL_Delay(50);
	  		  var_adc_2 = HAL_ADC_GetValue(&hadc2);
			  sprintf(buffer_adc, "%d", var_adc);
			  unsigned char err2[] = "data loi2\r\n";
			  HAL_UART_Transmit(&huart1, err2, strlen((char *)err2), 10);
			  sprintf(buffer_adc_2, "%d", var_adc_2);
	  		  for(int i=0;i<300;i++) {
	  			  total[i] =0;
	  		  }
	  		  strcat(total,tx_humi);
	  		  strcat(total,space);
	  		  strcat(total,tx);
	  		  strcat(total,space);
	  		  strcat(total,buffer_adc);
		  	  strcat(total,space);
		  	  strcat(total,buffer_adc_2);
	  		  strcat(total,end);
	  		  unsigned char data3[] = "AT+CMQTTCONNECT=0,\"tcp://test.mosquitto.org:1883\",60,0\r\n";
	  		  HAL_UART_Transmit(&huart1, data3, strlen((char *)data3), 10);
	  		  HAL_Delay(1000);

	  		  unsigned char data4[] = "AT+CMQTTTOPIC=0,5\r\n";
	  		  HAL_UART_Transmit(&huart1, data4, strlen((char *)data4), 10);
	  		  HAL_Delay(10);

	  		  unsigned char data5[] = "vonam\r\n";
	  		  HAL_UART_Transmit(&huart1, data5, strlen((char *)data5), 10);
	  		  HAL_Delay(1000);

	  		  unsigned char data6[] = "AT+CMQTTPAYLOAD=0,19\r\n";
	  		  HAL_UART_Transmit(&huart1, data6, strlen((char *)data6), 10);
	  		  HAL_Delay(10);

	  		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  		  HAL_UART_Transmit(&huart1, (unsigned char *)total, strlen((char *)total), 500);
	  		  HAL_Delay(1000);

	  		  unsigned char data8[] = "AT+CMQTTPUB=0,1,60,0,0\r\n";
	  		  HAL_UART_Transmit(&huart1, data8, strlen((char *)data8), 10);
	  		  HAL_Delay(1000);

	  		  unsigned char data9[] = "AT+CMQTTDISC=0,200\r\n";
	  		  HAL_UART_Transmit(&huart1, data9, strlen((char *)data9), 10);
	  		  HAL_Delay(1000);


	  	  }
	  	  HAL_Delay(1000);

	    }
  }
  /* USER CODE END 3 */


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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
