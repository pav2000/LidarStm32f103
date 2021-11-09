/*
 * lidar.h
 *
 *  Created on: Oct 28, 2021
 *      Author: Admin
 */

#ifndef INC_LIDAR_H_
#define INC_LIDAR_H_
#include "stm32f1xx_hal.h"
#include "st7735.h"
#include "fonts.h"

void scale_show(void);
void radar_show(uint16_t angle, uint16_t dist);
void readOnePoket(void);
void showData(void);



extern uint32_t time;


#endif /* INC_LIDAR_H_ */
