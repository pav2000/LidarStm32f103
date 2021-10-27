/*
 * st7735_cfg.h
 *
 *  Created on: 16 ���. 2019 �.
 *      Author: Andriy Honcharenko
 *        Blog: https://stm32withoutfear.blogspot.com
 */

#ifndef ST7735_CFG_H_
#define ST7735_CFG_H_

#include "main.h"

#define ST7735_SPI_PORT hspi1	//hspi1, hspi2, hspi3...
#define USE_SPI_DMA		    	//if used DMA for SPI bus

#define ST7735_1_8_DEFAULT_ORIENTATION	// AliExpress/eBay 1.8" display, default orientation
//#define ST7735S_1_8_DEFAULT_ORIENTATION 	// WaveShare ST7735S-based 1.8" display, default orientation
//#define ST7735_1_44_DEFAULT_ORIENTATION 	// 1.44" display, default orientation
//#define ST7735_MINI_DEFAULT_ORIENTATION 	// mini 160x80 display (it's unlikely you want the default orientation)

//Port and pin connected signal 'RES' (reset) ST7735 display
#ifndef ST7735_RES_Pin
#define ST7735_RES_Pin 			TFT_RST_Pin
#endif
#ifndef ST7735_RES_GPIO_Port
#define ST7735_RES_GPIO_Port 	TFT_RST_GPIO_Port
#endif
//Port and pin connected signal 'DC' (data or command) ST7735 display
#ifndef ST7735_DC_Pin
#define ST7735_DC_Pin 			TFT_DC_Pin
#endif
#ifndef ST7735_DC_GPIO_Port
#define ST7735_DC_GPIO_Port 	TFT_DC_GPIO_Port
#endif
//Port and pin connected signal 'CS' (chip select) ST7735 display
#ifndef ST7735_CS_Pin
#define ST7735_CS_Pin 			TFT_CS_Pin
#endif
#ifndef ST7735_CS_GPIO_Port
#define ST7735_CS_GPIO_Port 	TFT_CS_GPIO_Port
#endif
//Port and pin connected signal 'BL' (back light) ST7735 display
#ifndef ST7735_BL_Pin
#define ST7735_BL_Pin			TFT_LED_Pin
#endif
#ifndef ST7735_BL_GPIO_Port
#define ST7735_BL_GPIO_Port 	TFT_LED_GPIO_Port
#endif

#endif /* ST7735_CFG_H_ */
