/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "st7735.h"
#include "fonts.h"
#include "lidar.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum state_type   // Стадия приема пакета (без DMA)
	{
	START1,
	START2,
	HEADER,
	DATA
	} state;

typedef struct {  // Заголовок посылки
				  uint8_t pack_type;    // Тип пакета
				  uint8_t data_lenght;  // Длина данных (по три байта)
				  uint16_t start_angle; // Начальный угол
				  uint16_t stop_angle;  // Конечный угол
				  uint16_t temp ;       // Неизвестно
                } typeHeader;

typedef struct {  // Массив с точками 360 градусов его и выводим
					uint16_t distance;
					uint16_t quality;
					uint8_t x,y;  // Координаты точки на дисплее
					} dataPoint;
#pragma pack(push, 1)
typedef struct {  // Одно измерние приходящее из лидара
				uint8_t	quality;   // качество сигнала
				uint16_t distance; // дистанция
                } onePoint;
#pragma pack(pop)
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
// Глобальные настройки проекта
#define VERSION  "0.33"   // Версия программы
#define UART_DMA          // Использовать DMA для чтения данных (нет пропуска пакетов, скорость поднялась почти в 2 раза)
#define CONST_SCALE  65   // Базовый коэффициент масштабирования, полученная дистанция делится на (CONST_SCALE*scale)
#define AVERAGING         // Усреднение значений по одинаковым углам (актуально без DMA)
#define MAXZOOM      6    // Максимальный масштаб (scale)
#define CENTRE_X     64   // Координаты цента полярных координат для дисплея 160х128
#define CENTRE_Y     64   //
#define RADIUS       64   // Радиус экрана лидара для дисплея 160х128
#define RxBuf_SIZE   512  // Размер буферов для чтения
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern void scale_show(void);
extern void radar_show(uint16_t angle, uint16_t dist);
extern void readOnePoket(void);
extern void showData(void);
extern void showStartScreen(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TFT_LED_Pin GPIO_PIN_1
#define TFT_LED_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOB
#define TFT_CS_Pin GPIO_PIN_13
#define TFT_CS_GPIO_Port GPIOB
#define TFT_DC_Pin GPIO_PIN_14
#define TFT_DC_GPIO_Port GPIOB
#define TFT_RST_Pin GPIO_PIN_15
#define TFT_RST_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_4
#define BUZZER_GPIO_Port GPIOB
#define ENC_BTN_Pin GPIO_PIN_5
#define ENC_BTN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
