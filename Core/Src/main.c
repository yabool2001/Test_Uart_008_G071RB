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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_TX_TIMEOUT				100
#define UART_RX_TIMEOUT				300
#define UART_RX_XL_TIMEOUT			5000
#define RX_BUFF_SIZE				50
#define TX_BUFF_SIZE				60
#define PW_BUFF_SIZE				5
#define GN_BUFF_SIZE				35
#define TD_PAYLOAD_BUFF_SIZE		90
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim14; // 1  second  timer
TIM_HandleTypeDef htim16; // 10 seconds timer

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t				checklist				= 0 ;
uint8_t				tim14_on				= 0 ;
uint8_t				tim16_on	 			= 0 ;

HAL_StatusTypeDef   uart_status ;
uint8_t             rx_buff[RX_BUFF_SIZE] ;
uint8_t             tx_buff[TX_BUFF_SIZE] ;
char				pw_buff[PW_BUFF_SIZE] ;
char				gn_buff[GN_BUFF_SIZE] ;
char 				td_at_comm[TD_PAYLOAD_BUFF_SIZE] ;
char* 				chunk ;
uint16_t			rx_len =  RX_BUFF_SIZE ;// stosowane tylko HAL_UARTEx_ReceiveToIdle()

char                hello[]         		= "Hello! Test_Swarm_001_G071RB\n" ;

// SWARM AT Commands
const char*			gn_mostrecent_at_comm	= "$GN @*67\n" ;

// SWARM AT Answers
const char*         gn_mostrecent_answer	= "$GN 52.2782,20.8089,84,359,2*2b\n" ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void 		send2uart 				( UART_HandleTypeDef* , const char* , const char* ) ;
void		nulling_array 			( uint8_t* array , size_t size ) ;
void 		wait_for_tim14x			( uint8_t ) ;
void 		wait_for_tim16x			( uint8_t ) ;
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //__HAL_TIM_CLEAR_IT ( &htim14 , TIM_IT_UPDATE ) ; // żeby nie generować przerwania TIM6 od razu: https://stackoverflow.com/questions/71099885/why-hal-tim-periodelapsedcallback-gets-called-immediately-after-hal-tim-base-sta
  //__HAL_TIM_CLEAR_IT ( &htim16 , TIM_IT_UPDATE ) ; // żeby nie generować przerwania TIM6 od razu: https://stackoverflow.com/questions/71099885/why-hal-tim-periodelapsedcallback-gets-called-immediately-after-hal-tim-base-sta

  uart_status = HAL_UART_Transmit ( &huart2 , (const uint8_t *) hello , strlen ( hello ) , UART_TX_TIMEOUT ) ;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* I found couple of threads that deal with this situation and they suggested to implement HAL_UART_ErrorCallback() and clear the ORE.
	   * https://community.st.com/s/question/0D50X00009ZEOZ3SAP/nucleo-f767zi-system-freeze-upon-uart-ore-interrupt
	   */
	  //__HAL_UART_CLEAR_OREFLAG ( &huart2 ) ;
	  //__HAL_UART_CLEAR_IDLEFLAG ( &huart2 ) ;
	  send2uart ( &huart2 , gn_mostrecent_at_comm , gn_mostrecent_answer ) ;
	  //__HAL_UART_CLEAR_OREFLAG ( &huart2 ) ;
	  //__HAL_UART_CLEAR_IDLEFLAG ( &huart2 ) ;
	  wait_for_tim14x ( 2 ) ;
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 30, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 16000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 16000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GREEN_Pin */
  GPIO_InitStruct.Pin = GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GREEN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void send2uart ( UART_HandleTypeDef* huart , const char* at_command , const char* answer )
{
	/*
	1. sposób zastosować malloc
	2. upewnić się, że wszędzie gdzie trzeba mam \0 na końcu
	3. albo coś z tymi cast jest nie tak, które namiętnie stosuję
	*/

	uint8_t* uart_rx_buff = malloc ( 99 * sizeof (uint8_t) ) ;
	uint8_t* uart_tx_buff = malloc ( 99 * sizeof (uint8_t) ) ;

	sprintf ( (char*) uart_tx_buff , "%s" , at_command ) ;
	__HAL_UART_SEND_REQ ( huart , UART_RXDATA_FLUSH_REQUEST ) ; //https://community.st.com/s/question/0D53W00000oXKU2SAO/efficient-way-to-process-usartreceived-data-and-flush-rx-buffer-
	uart_status = HAL_UART_Transmit ( huart , (const uint8_t *) uart_tx_buff ,  strlen ( (char*) uart_tx_buff ) , UART_TX_TIMEOUT ) ;
	uart_status = HAL_UART_Receive ( huart , uart_rx_buff , /*sizeof ( uart_rx_buff )*/99 , UART_RX_TIMEOUT ) ;
	if ( strncmp ( (char*) uart_rx_buff , answer , strlen ( answer ) ) == 0 )
	{
		sprintf ( (char*) uart_tx_buff , "Yes. %s" , uart_rx_buff ) ;
		uart_status = HAL_UART_Transmit ( huart , (const uint8_t *) uart_tx_buff ,  strlen ( (char*) uart_rx_buff ) , UART_TX_TIMEOUT ) ;
	}
	else
	{
		//sprintf ( (char*) tx_buff , "No. %s" , rx_buff ) ;
		//uart_status = HAL_UART_Transmit ( huart , (const uint8_t *) tx_buff ,  strlen ( (char*) tx_buff ) , UART_TX_TIMEOUT ) ;
	}
	free ( uart_rx_buff ) ;
	free ( uart_tx_buff ) ;
	//nulling_array ( tx_buff , sizeof ( tx_buff ) ) ;
	//nulling_array ( rx_buff , sizeof ( rx_buff ) ) ;
}

void nulling_array ( uint8_t* array , size_t size )
{
	for ( uint8_t i = 0 ; i < size ; i++ ) { array[i] = 0 ; }
}

void wait_for_tim14x ( uint8_t x )
{
	uint8_t i ;
	for ( i = 0 ; i < x ; i++ )
	{
		tim14_on = 1 ;
		HAL_TIM_Base_Start_IT ( &htim14 ) ;
		while ( tim14_on )
			__NOP () ;
	}
}

void wait_for_tim16x ( uint8_t x )
{
	uint8_t i ;
	for ( i = 0 ; i < x ; i++ )
	{
		tim16_on = 1 ;
		HAL_TIM_Base_Start_IT ( &htim16 ) ;
		while ( tim16_on )
			__NOP () ;
	}
}

void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef *htim )
{
	if ( htim->Instance == TIM14 )
	{
		tim14_on = 0 ;
		HAL_TIM_Base_Stop_IT ( &htim14 ) ;
	}
	if ( htim->Instance == TIM16 )
	{
		tim16_on = 0 ;
		HAL_TIM_Base_Stop_IT ( &htim16 ) ;
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
