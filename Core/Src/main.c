/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include "DS18B20.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum option_t{
	TRANSPARENT,
	POINT2POINT
} option_t;

typedef enum e32_lora_mode_t{
	GENERAL,
	POWER_SAVING,
	WAKE_UP,
	SLEEP
} e32_lora_mode_t;

typedef enum set_mode_state_t{
	SUCCESSED,
	FAILED
} set_mode_state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
	float temp;
	DS18B20_Name DS1;
	uint8_t temp_tenth, temp_unit, temp_decimal;


	set_mode_state_t state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

set_mode_state_t set_mode_e32_lora(e32_lora_mode_t mode,
					   GPIO_TypeDef *GPIOx_AUX, uint16_t GPIO_Pin_AUX,
					   GPIO_TypeDef *GPIOx_M0, uint16_t GPIO_Pin_M0,
					   GPIO_TypeDef *GPIOx_M1, uint16_t GPIO_Pin_M1
						)
{
	GPIO_PinState idle = HAL_GPIO_ReadPin(GPIOx_AUX, GPIO_Pin_AUX);
	/* Check idle state of module  */
	while(idle != GPIO_PIN_SET) // busy
	{

	};
	HAL_Delay(5);

	/* Set mode */
	if(idle == GPIO_PIN_SET)
	{
		switch(mode)
			{
				case GENERAL:
					HAL_GPIO_WritePin(GPIOx_M1, GPIO_Pin_M1, GPIO_PIN_RESET); // M1 = 0
					HAL_GPIO_WritePin(GPIOx_M0, GPIO_Pin_M0, GPIO_PIN_RESET); // M0 = 0
					HAL_Delay(10);

					break;
				case POWER_SAVING:
					HAL_GPIO_WritePin(GPIOx_M1, GPIO_Pin_M1, GPIO_PIN_RESET);// M1 = 0
					HAL_GPIO_WritePin(GPIOx_M0, GPIO_Pin_M0, GPIO_PIN_SET); // M0 = 1
					HAL_Delay(10);
					break;
				case WAKE_UP:
					HAL_GPIO_WritePin(GPIOx_M1, GPIO_Pin_M1, GPIO_PIN_SET); // M1 = 1
					HAL_GPIO_WritePin(GPIOx_M0, GPIO_Pin_M0, GPIO_PIN_RESET); // M0 = 0
					HAL_Delay(10);
					break;
				case SLEEP:
					HAL_GPIO_WritePin(GPIOx_M1, GPIO_Pin_M1, GPIO_PIN_SET); // M1 = 1
					HAL_GPIO_WritePin(GPIOx_M0, GPIO_Pin_M0, GPIO_PIN_SET); // M0 = 1
					HAL_Delay(10);

					break;
				default:
					HAL_GPIO_WritePin(GPIOx_M1, GPIO_Pin_M1, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOx_M0, GPIO_Pin_M0, GPIO_PIN_RESET);
					HAL_Delay(10);
			}

	};

	/* wait 3ms for AUX high */
	while(idle != GPIO_PIN_SET)
		{

		};
	HAL_Delay(5);
	if (idle == GPIO_PIN_SET)
	{
		return SUCCESSED;
	}
	else
	{
		return FAILED;
	}
}





void config_e32_lora(UART_HandleTypeDef *huart, int baudrate,
					 uint8_t add_high_byte, uint8_t add_low_byte, option_t option)
{
	uint8_t SPEED;
	uint8_t OPTION;
	uint8_t ADDH = add_high_byte;
	uint8_t ADDL = add_low_byte;
	uint8_t reset_config_cmd[] = {0xC1, 0xC1, 0xC1};

	/* 433MHz */
	uint8_t CHAN = 0x17;

	/* Reset module Command */
	HAL_UART_Transmit(huart, reset_config_cmd, sizeof(reset_config_cmd), 100);
	HAL_Delay(10);

	switch(baudrate)
	{
		case 1200:
			SPEED = 0x02;
			break;
		case 2400:
			SPEED = 0x0A;
			break;
		case 4800:
			SPEED = 0x12;
			break;
		case 9600:
			SPEED = 0x1A;
			break;
		case 19200:
			SPEED = 0x22;
			break;
		case 38400:
			SPEED = 0x2A;
			break;
		case 57600:
			SPEED = 0x32;
			break;
		case 115200:
			SPEED = 0x3A;
			break;
		default:
			SPEED = 0x1A;
	};

	switch(option)
	{
		case TRANSPARENT:
			OPTION = 0x40 ;
			break;
		case POINT2POINT:
			OPTION = 0xC0;
			break;
		default:
			OPTION = 0x40;
	};

	uint8_t config_cmd[] = {0xC0, ADDH, ADDL, SPEED, CHAN, OPTION};

	/* Configure */

	HAL_UART_Transmit(huart, config_cmd, sizeof(config_cmd), 100);
	HAL_Delay(150);
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  state = set_mode_e32_lora(SLEEP, GPIOA, GPIO_PIN_3, GPIOA, GPIO_PIN_1, GPIOA, GPIO_PIN_2);

  while(state != SUCCESSED)
  {
	  state = set_mode_e32_lora(SLEEP, GPIOA, GPIO_PIN_3, GPIOA, GPIO_PIN_1, GPIOA, GPIO_PIN_2);
  }

  config_e32_lora(&huart1, 9600, 0x00, 0x05, POINT2POINT); // config


  set_mode_e32_lora(GENERAL, GPIOA, GPIO_PIN_3, GPIOA, GPIO_PIN_1, GPIOA, GPIO_PIN_2);

  //uint8_t Tx_Buffer[] = {0x00, 0x20, 0x17, 50, 80, 60};

  DS18B20_Init(&DS1, &htim4, DS18B20_GPIO_Port, DS18B20_Pin);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 temp = DS18B20_ReadTemp(&DS1);
	 temp_tenth 	= ((int) temp)/10;
	 temp_unit  	= ((int) temp)%10;
	 temp_decimal 	= (int) ((temp - 10*temp_tenth - temp_unit)*10);
	 uint8_t Tx_Buffer[] = {0x00, 0x25, 0x17, temp_tenth + 48, temp_unit + 48, temp_decimal + 48};
	 HAL_UART_Transmit(&huart1, Tx_Buffer , sizeof(Tx_Buffer), 100);
	 HAL_Delay(50);

	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	 HAL_Delay(2000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MD0_Pin|MD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MD0_Pin MD1_Pin */
  GPIO_InitStruct.Pin = MD0_Pin|MD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : AUX_Pin */
  GPIO_InitStruct.Pin = AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AUX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DS18B20_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

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
