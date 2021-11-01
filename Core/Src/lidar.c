
#include "lidar.h"
#include "cmsis_os.h"
extern UART_HandleTypeDef huart2;

enum state_type   // Стадия приема пакета
	{
	START0,
	START1,
	START2,
	HEADER,
	DATA
	} state;

uint8_t show=0;  // Флаг полного оборота
uint16_t angle_old=0;                      // старый угол (0-359)
uint8_t  xLine=CENTRE_X;yLine=CENTRE_X;    // текущие коордианты линии
uint8_t  xPoint=CENTRE_X;yPoint=CENTRE_X;  // текущие коордианты расстояния
char rxBuf[512];
struct dataPoint{
	uint16_t distance;
	uint16_t quality;
	} data[360+1];


// таблица синусов и косинусов через градус для ускорения
const uint16_t sin1000[90]={0,17,35,52,70,87,105,122,139,156,174,191,208,225,242,259,276,292,309,326,342,358,375,391,407,423,438,454,469,485,500,515,
		                    530,545,559,574,588,602,616,629,643,656,669,682,695,707,719,731,743,755,766,777,788,799,809,819,829,839,848,857,866,875,
				            883,891,899,906,914,920,927,934,940,946,951,956,961,966,970,974,978,982,985,988,990,993,995,996,998,999,999,1000};
const uint16_t cos1000[90]={1000,999,999,998,996,995,993,990,988,985,982,978,974,970,966,961,956,951,946,940,934,927,921,914,906,899,891,883,875,866,
		                    857,848,839,829,819,809,799,788,777,766,755,743,731,719,707,695,682,669,656,643,629,616,602,588,574,559,545,530,515,500,
							485,469,454,438,423,407,391,375,358,342,326,309,292,276,259,242,225,208,191,174,156,139,122,105,87,70,52,35,17,0};

// Показ радара, смена положения луча и добавление еще одной точки
// Вход теущая дистанция
void radar_show(uint16_t angle, uint16_t dist)
{
	uint16_t dt=0;
	char buf[8];
	uint32_t  x1,y1;
	if(dist>RADIUS) dist=RADIUS;
	ST7735_DrawLine(CENTRE_X, CENTRE_Y,xLine, yLine, ST7735_BLACK);	// Стереть старую линию
	ST7735_DrawPixel(xPoint, yPoint, ST7735_YELLOW);                // Восстановить точку
// Расчет новой конечной точки
// В зависимости от квадранта угла
if ((angle>=0)&&(angle<90))    {x1=CENTRE_X+(RADIUS*sin1000[angle])/1000;
                                y1=CENTRE_Y - (RADIUS*cos1000[angle])/1000;
                                xPoint=CENTRE_X+(dist*sin1000[angle])/1000;
                                yPoint=CENTRE_Y - (dist*cos1000[angle])/1000;} else
if ((angle>=90)&&(angle<180))  {x1=CENTRE_X+(RADIUS*cos1000[angle-90])/1000;
                                y1=CENTRE_Y + (RADIUS*sin1000[angle-90])/1000;
                                xPoint=CENTRE_X+(dist*cos1000[angle-90])/1000;
                                yPoint=CENTRE_Y + (dist*sin1000[angle-90])/1000;} else
if ((angle>=180)&&(angle<270)) {x1=CENTRE_X-(RADIUS*sin1000[angle-180])/1000;
                                y1=CENTRE_Y + (RADIUS*cos1000[angle-180])/1000;
                                xPoint=CENTRE_X-(dist*sin1000[angle-180])/1000;
                                yPoint=CENTRE_Y + (dist*cos1000[angle-180])/1000;}else
if ((angle>=270)&&(angle<360)) {x1=CENTRE_X-(RADIUS*cos1000[angle-270])/1000;
                                y1=CENTRE_Y - (RADIUS*sin1000[angle-270])/1000;
                                xPoint=CENTRE_X-(dist*cos1000[angle-270])/1000;
                                yPoint=CENTRE_Y - (dist*sin1000[angle-270])/1000;}

ST7735_DrawLine(CENTRE_X, CENTRE_Y,x1, y1, ST7735_GREEN);	// Новая линия
ST7735_DrawPixel(xPoint, yPoint, ST7735_RED);
xLine=x1;
yLine=y1;
angle_old=angle+3;    // Можно менять шаг в градусах
if (angle>=360) { // Полный круг
	angle=0;
	dt=HAL_GetTick()-time; // Время полного круга (измерение)
	time=HAL_GetTick();
	itoa(dt,buf,10);
	ST7735_DrawString(110, 118, buf, Font_7x10, ST7735_WHITE, ST7735_BLACK);
	ST7735_DrawString(145, 118, "ms", Font_7x10, ST7735_WHITE, ST7735_BLACK);
    }
}

// Показ Шкалы
void scale_show(void)
{
uint8_t i;
	ST7735_DrawFastVLine(2*CENTRE_X+4,0, CENTRE_Y, ST7735_WHITE);
	for (i=0;i<CENTRE_Y/10;i++)
		ST7735_DrawFastHLine(2*CENTRE_X+4,3+10*i, 4, ST7735_WHITE);
	ST7735_DrawString(2*CENTRE_X+10, 0*10, "6", Font_7x10, ST7735_RED, ST7735_BLACK);
	ST7735_DrawString(2*CENTRE_X+10, 1*10, "5", Font_7x10, ST7735_RED, ST7735_BLACK);
	ST7735_DrawString(2*CENTRE_X+10, 2*10, "4", Font_7x10, ST7735_RED, ST7735_BLACK);
	ST7735_DrawString(2*CENTRE_X+10, 3*10, "3", Font_7x10, ST7735_RED, ST7735_BLACK);
	ST7735_DrawString(2*CENTRE_X+10, 4*10, "2", Font_7x10, ST7735_RED, ST7735_BLACK);
	ST7735_DrawString(2*CENTRE_X+10, 5*10, "1", Font_7x10, ST7735_RED, ST7735_BLACK);
}

void readOnePoket(void)
{

	  uint8_t pack_type;
	  int16_t data_lenght;
	  uint16_t start_angle;
	  uint16_t stop_angle;
	  int32_t diff;
	  int16_t angle_per_sample;
	  int16_t counter;
	  uint16_t i;
//	  uint8_t show=0;  // Флаг полного оборота
	  //  HAL_UART_Receive_IT(&huart2,(uint8_t*) rxBuf,60);
	  //  HAL_UART_Receive(&huart2, rxBuf, sizeof(rxBuf), HAL_MAX_DELAY);

	  state = START1;  // Начальная стадия
	   counter=0;
	   while (1)
	  {
	   if (state == START1)   // Поиск заголовка из двух байт
	   {
		   HAL_UART_Receive(&huart2, rxBuf, 1, HAL_MAX_DELAY);
		   if (rxBuf[0]==0xAA) { state = START2; } else { continue; /* Синхронизация 1*/}
	   }
	   else if (state == START2)
	   {
		   HAL_UART_Receive(&huart2, rxBuf, 1, HAL_MAX_DELAY);
		   if (rxBuf[0]==0x55) { state = HEADER; } else { state = START1; continue;/* Синхронизация 2*/}
	   }
	   else if (state == HEADER) {  // Разбор заголовка посылки
		    HAL_UART_Receive(&huart2, rxBuf, 8, HAL_MAX_DELAY);
			pack_type = rxBuf[0];                                                    // Тип посылки (тип лидара???)
			data_lenght = rxBuf[1];                                                  // Число измерений в посылке
			start_angle = (rxBuf[3] << 8) + rxBuf[2];                                // Начальный угол посылки
			stop_angle  = (rxBuf[5] << 8) + rxBuf[4];                                // Конечный угол посылки
			diff = stop_angle - start_angle;                                         // Диапазон углов
			if (stop_angle < start_angle) diff =  0xB400 - start_angle + stop_angle; // если перешли через 0  (0xB400 скорее всего максимальный угол)
			angle_per_sample = 0;                                                    // угол одного образца
			if (diff > 1) angle_per_sample = diff / (data_lenght-1);                 // вычисление изменения угла на одно измерение в посылке
			counter ++;
			if (pack_type!= 0x38) { counter = 0; }                                   // не тот тип пакета/лидара
			state = DATA;
			continue;
	   }
	   else if (state == DATA) {                                                     // Чтение измерений в пакете
		   uint32_t index;                                                           // Угол в градусах, он же индекс массива данных
		   int32_t data0,data1,data2,angle;
		   state = START1;
	//	   HAL_UART_Receive(&huart2, rxBuf, data_lenght * 3, HAL_MAX_DELAY);        // читаем все данные
		   HAL_UART_Receive_IT(&huart2, rxBuf, data_lenght * 3);        // читаем все данные
		   osDelay(8);
		   HAL_GPIO_TogglePin(GPIOB, LED2_Pin); // Инвертирование состояния выхода.
	       for (i=0;i<data_lenght;i++){                                             // По всем измерениям пакета
	    	    data0 = rxBuf[i*3 + 0];
				data1 = rxBuf[i*3 + 1];
				data2 = rxBuf[i*3 + 2];

				index = (start_angle + angle_per_sample * i)*360/0xB400;            // расчет угла в градусах
				if (index>359) { index=index-359; show=1;}                          // переход через 0 и признак необходимости показа

				data[index].distance = (data2  << 8) + data1;
				data[index].quality=data0;

		//		data[i].angle = (start_angle + angle_per_sample * i);
		//		data[i].angle = (-3.1415/2) * (angle / 0xB400);
				//data[i].distance_list.append(distance / 1000) # div to convert mm to meter
		//		 radar_show(data[i].angle/100,data[i].distance/1000);
	       }// for
	 //      HAL_GPIO_TogglePin(GPIOB, LED2_Pin); // Инвертирование состояния выхода.
	 //      showData();
//       if (show==1){
//	    	   show=0;
//	        for (i=0;i<360;i++){ // Показ графика
//	      			 radar_show(i,data[i].distance/100);
//	      			data[i].distance=0;
//	            }// for
//	      } // if
	   }
	  }
}

void showData(void){
uint16_t i;
if (show==1){
 	   show=0;
     for (i=0;i<360;i++){ // Показ графика
   			 radar_show(i,data[i].distance/50);
   			data[i].distance=0;
         }// for
   }
}
