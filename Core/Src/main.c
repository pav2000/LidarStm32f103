/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* Definitions for readLidar */
osThreadId_t readLidarHandle;
uint32_t readLidarBuffer[ 128 ];
osStaticThreadDef_t readLidarControlBlock;
const osThreadAttr_t readLidar_attributes = {
  .name = "readLidar",
  .cb_mem = &readLidarControlBlock,
  .cb_size = sizeof(readLidarControlBlock),
  .stack_mem = &readLidarBuffer[0],
  .stack_size = sizeof(readLidarBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for showLidar */
osThreadId_t showLidarHandle;
uint32_t showLidarBuffer[ 128 ];
osStaticThreadDef_t showLidarControlBlock;
const osThreadAttr_t showLidar_attributes = {
  .name = "showLidar",
  .cb_mem = &showLidarControlBlock,
  .cb_size = sizeof(showLidarControlBlock),
  .stack_mem = &showLidarBuffer[0],
  .stack_size = sizeof(showLidarBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
dataPoint data[360+1];     // массив данных лидара
uint8_t scale=1;           // текущий масштаб графика от 1 до 6
uint8_t fScale=0;          // Необходимость перерисовать шкалу (изменение масштаба)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Пищалка
void beep(uint16_t t)
{
	HAL_GPIO_TogglePin(GPIOB, BUZZER_Pin); // Звук переключения
	HAL_Delay(t);
	HAL_GPIO_TogglePin(GPIOB, BUZZER_Pin);
}
#ifdef UART_DMA
	uint8_t RxBuf0[RxBuf_SIZE];  // два буфера работают попеременно, пока один пишется другой разбирается
	uint8_t RxBuf1[RxBuf_SIZE];
	uint8_t *RxBuf;              // Указатель на текущий буфер для чтения

	void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
	{
	int32_t diff;
	int16_t angle_per_sample;
	uint32_t index;          // Угол в градусах, он же индекс массива данных
	typeHeader  *header;     // указатель на заголовок пакета
	uint8_t *Buf;            // Указатель  на буфер для разбора
	onePoint *point;         // Указатель на одно измерение приходящее с лидара
	int i,j;
		if (huart->Instance == USART2)
		{
		Buf=RxBuf;   // получить ссылку где лежат последние данные для последующей обработки
		if (RxBuf==RxBuf0) RxBuf=RxBuf1; else RxBuf=RxBuf0; // переключиться на другой буфер для чтения
			/* start the DMA again */
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) RxBuf, RxBuf_SIZE);  // Чтение до idle это важно!
			__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
			HAL_GPIO_TogglePin(GPIOB, LED2_Pin);                                   // Инвертирование состояния светодиода - чтение нового пакета

    // Разбор полученных данных они лежат в Buf
	for(i=0;i<Size;i++)  // По всему принятому буферу
		{
		if((Buf[i+0]==0xAA)&&(Buf[i+1]==0x55)){    // найден заголовок
		header=(typeHeader*)&Buf[i+2];             // наложить структуру заголовка на принятые данные
//		if(header->pack_type!=0x38) break;         // не правильный пакет
		diff = header->stop_angle - header->start_angle;                           // Диапазон углов
		if (header->stop_angle<header->start_angle) diff=0xB400-header->start_angle+header->stop_angle; // если перешли через 0  (0xB400 скорее всего максимальный угол)
		angle_per_sample = 0;                                                    // угол одного образца
		if (diff > 1) angle_per_sample = diff / (header->data_lenght-1);          // вычисление изменения угла на одно измерение в посылке

		// Чтение измерений
    	//	   if(header->data_lenght>40) break;
		   if ((10+header->data_lenght*3)>Size) break;                             // Это ошибка длины принятого пакета, пропускаем пакет
		   for (j=0;j<header->data_lenght;j++){                                    // По всем измерениям пакета
			   index = (header->start_angle + angle_per_sample * j)*360/0xB400;    // расчет угла в градусах
			   if (index>359) index=index-359;                                     // переход через 0
			   if (index>359) index=359;
			   point=(onePoint*)&Buf[i+10+j*3];                                     // наложить структуру одного измерения на буфер с данными
			   data[index].distance=point->distance;
			   data[index].quality=point->quality;
		       } // for
    		} // if((RxBuf[i+0]==0xAA)&&(RxBuf[i+1]==0x55))
		break;
		} // for

		} // if (huart->Instance == USART2)
	}
#endif

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
   HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);    // Установить светодиод 2 в 0
   beep(200);
   showStartScreen();

   #ifdef UART_DMA
   RxBuf=RxBuf0; // Установить текущий буфер
   HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);  // Запустить чтение с uart лидара
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);         //
   #endif
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of readLidar */
  readLidarHandle = osThreadNew(StartDefaultTask, NULL, &readLidar_attributes);

  /* creation of showLidar */
  showLidarHandle = osThreadNew(StartTask02, NULL, &showLidar_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
// }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 153600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_RX;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TFT_LED_GPIO_Port, TFT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|LED1_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TFT_CS_Pin|TFT_DC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : TFT_LED_Pin TFT_RST_Pin */
  GPIO_InitStruct.Pin = TFT_LED_Pin|TFT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED1_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_CS_Pin TFT_DC_Pin */
  GPIO_InitStruct.Pin = TFT_CS_Pin|TFT_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_BTN_Pin */
  GPIO_InitStruct.Pin = ENC_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENC_BTN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
 #ifdef UART_DMA
	 uint8_t pressKey=0;      // Отпускание клавиши
 #endif
  /* Infinite loop */
  for(;;)
  {
  #ifndef UART_DMA // Чтение пакета и чтение кнопки
   readOnePoket();
   osDelay(1);
  #else  // DMA - пакеты читаются по событию idle только чтение кнопки
   if (HAL_GPIO_ReadPin(GPIOB, ENC_BTN_Pin)==0) pressKey=0;  // Клавиша отпущена
   osDelay(30);
   // Чтение кнопки энкодера - изменение масштаба
	 if ((HAL_GPIO_ReadPin(GPIOB, ENC_BTN_Pin)==1)&&(pressKey==0)) {
		 osDelay(30);
		 if (HAL_GPIO_ReadPin(GPIOB, ENC_BTN_Pin)==1){
			if(scale<MAXZOOM) scale++; else scale=1;
			pressKey=1;
			fScale=1; // Надо перерисовать шкалу
		 }
	 }
	 osDelay(50);
  #endif
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the showLidar thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  showRadar();
 //   osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
