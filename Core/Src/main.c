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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define VERSION  "0.1"   // Версия программы
#define CENTRE_X 64      // Координаты цента полярных координат
#define CENTRE_Y 64      //
#define RADIUS 64      //

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
beep(uint16_t t)
{
	HAL_GPIO_TogglePin(GPIOB, BUZZER_Pin); // Звук переключения
	HAL_Delay(t);
	HAL_GPIO_TogglePin(GPIOB, BUZZER_Pin);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
   HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);    // Установить светодиод 2 в 0
   beep(200);
   ST7735_Init();  // Не забываем в кубе настроить DMA
   ST7735_Backlight_On(); // Включить подсветку дисплея
   ST7735_SetRotation(3);
   ST7735_FillScreen(ST7735_BLACK);
   ST7735_DrawString(0, 108, "Hardware version: 1.3", Font_7x10, ST7735_RED, ST7735_BLACK);
   ST7735_DrawString(0, 118, "Test prog version:", Font_7x10, ST7735_RED, ST7735_BLACK);
   ST7735_DrawString(130, 118, VERSION, Font_7x10, ST7735_RED, ST7735_BLACK);
   HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);    // Установить светодиод 2 в 1
   HAL_Delay(1000);
   ST7735_FillScreen(ST7735_BLACK);
   ST7735_DrawCircle(CENTRE_X, CENTRE_Y, RADIUS, ST7735_WHITE);
   scale_show();
   /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//  HAL_GPIO_TogglePin(GPIOB, LED2_Pin); // Инвертирование состояния выхода.
	//  radar_show((rand()%100+2)/2);
	  radar_show(45);
	//  HAL_Delay(100);                       // Пауза 50 миллисекунд.
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, TFT_CS_Pin|TFT_DC_Pin|TFT_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TFT_LED_GPIO_Port, TFT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TFT_CS_Pin TFT_DC_Pin */
  GPIO_InitStruct.Pin = TFT_CS_Pin|TFT_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TFT_LED_Pin TFT_RST_Pin */
  GPIO_InitStruct.Pin = TFT_LED_Pin|TFT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint16_t angle=0;                          // текущий угол (0-359)
uint8_t  xLine=CENTRE_X;yLine=CENTRE_X;    // текущие коордианты линии
uint8_t  xPoint=CENTRE_X;yPoint=CENTRE_X;  // текущие коордианты лидара


const uint16_t sin1000[90]={0,17,35,52,70,87,105,122,139,156,174,191,208,225,242,259,276,292,309,326,342,358,375,391,407,423,438,454,469,485,500,515,
		                    530,545,559,574,588,602,616,629,643,656,669,682,695,707,719,731,743,755,766,777,788,799,809,819,829,839,848,857,866,875,
				            883,891,899,906,914,920,927,934,940,946,951,956,961,966,970,974,978,982,985,988,990,993,995,996,998,999,999,1000};
const uint16_t cos1000[90]={1000,999,999,998,996,995,993,990,988,985,982,978,974,970,966,961,956,951,946,940,934,927,921,914,906,899,891,883,875,866,
		                    857,848,839,829,819,809,799,788,777,766,755,743,731,719,707,695,682,669,656,643,629,616,602,588,574,559,545,530,515,500,
							485,469,454,438,423,407,391,375,358,342,326,309,292,276,259,242,225,208,191,174,156,139,122,105,87,70,52,35,17,0};

// Показ радара, смена положения луча и добавление еще одной точки
// Вход теущая дистанция
void radar_show(uint16_t dust)
{
	ST7735_DrawLine(CENTRE_X, CENTRE_Y,xLine, yLine, ST7735_BLACK);	// Стереть старую линию
	ST7735_DrawPixel(xPoint, yPoint, ST7735_YELLOW);
// Расчет новой конечной точки
uint32_t  x1,y1;
// В зависимости от квадранта угла
if ((angle>=0)&&(angle<90))    {x1=CENTRE_X+(RADIUS*sin1000[angle])/1000;
                                y1=CENTRE_Y - (RADIUS*cos1000[angle])/1000;
                                xPoint=CENTRE_X+(dust*sin1000[angle])/1000;
                                yPoint=CENTRE_Y - (dust*cos1000[angle])/1000;} else
if ((angle>=90)&&(angle<180))  {x1=CENTRE_X+(RADIUS*cos1000[angle-90])/1000;
                                y1=CENTRE_Y + (RADIUS*sin1000[angle-90])/1000;
                                xPoint=CENTRE_X+(dust*cos1000[angle-90])/1000;
                                yPoint=CENTRE_Y + (dust*sin1000[angle-90])/1000;} else
if ((angle>=180)&&(angle<270)) {x1=CENTRE_X-(RADIUS*sin1000[angle-180])/1000;
                                y1=CENTRE_Y + (RADIUS*cos1000[angle-180])/1000;
                                xPoint=CENTRE_X-(dust*sin1000[angle-180])/1000;
                                yPoint=CENTRE_Y + (dust*cos1000[angle-180])/1000;}else
if ((angle>=270)&&(angle<360)) {x1=CENTRE_X-(RADIUS*cos1000[angle-270])/1000;
                                y1=CENTRE_Y - (RADIUS*sin1000[angle-270])/1000;
                                xPoint=CENTRE_X-(dust*cos1000[angle-270])/1000;
                                yPoint=CENTRE_Y - (dust*sin1000[angle-270])/1000;}

ST7735_DrawLine(CENTRE_X, CENTRE_Y,x1, y1, ST7735_GREEN);	// Новая линия
ST7735_DrawPixel(xPoint, yPoint, ST7735_RED);
xLine=x1;
yLine=y1;
angle=angle+3;
if (angle>=360) angle=0;
}
// Показ Шкалы
// Вход теущая дистанция
void scale_show(void)
{
uint8_t i;
	ST7735_DrawFastVLine(2*CENTRE_X+2,0, CENTRE_Y, ST7735_WHITE);
	for (i=0;i<CENTRE_Y/10;i++)
		ST7735_DrawFastHLine(2*CENTRE_X+2,3+10*i, 4, ST7735_WHITE);
	ST7735_DrawString(2*CENTRE_X+10, 0*10, "6", Font_7x10, ST7735_RED, ST7735_BLACK);
	ST7735_DrawString(2*CENTRE_X+10, 1*10, "5", Font_7x10, ST7735_RED, ST7735_BLACK);
	ST7735_DrawString(2*CENTRE_X+10, 2*10, "4", Font_7x10, ST7735_RED, ST7735_BLACK);
	ST7735_DrawString(2*CENTRE_X+10, 3*10, "3", Font_7x10, ST7735_RED, ST7735_BLACK);
	ST7735_DrawString(2*CENTRE_X+10, 4*10, "2", Font_7x10, ST7735_RED, ST7735_BLACK);
	ST7735_DrawString(2*CENTRE_X+10, 5*10, "1", Font_7x10, ST7735_RED, ST7735_BLACK);


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
