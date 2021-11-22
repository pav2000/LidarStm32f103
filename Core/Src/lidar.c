
#include "main.h"
#include "lidar.h"
#include "cmsis_os.h"
#include "image.h"

extern UART_HandleTypeDef huart2;
extern dataPoint data[360+1];
extern uint8_t scale;                               // текущий масштаб графика
extern uint8_t fScale;                              // Необходимость перерисовать шкалу (изменение масштаба)

uint8_t  xLine=CENTRE_X,yLine=CENTRE_X;             // текущие коордианты линии
uint8_t  xPoint=CENTRE_X,yPoint=CENTRE_X,zPoint=0;  // текущие коордианты расстояния и цвет точки
#ifndef UART_DMA
   char rxBuf[RxBuf_SIZE];                          // Буфер для чтения без DMA
#endif


// таблица синусов и косинусов через градус для ускорения
const uint16_t sin1000[90]={0,17,35,52,70,87,105,122,139,156,174,191,208,225,242,259,276,292,309,326,342,358,375,391,407,423,438,454,469,485,500,515,
		                    530,545,559,574,588,602,616,629,643,656,669,682,695,707,719,731,743,755,766,777,788,799,809,819,829,839,848,857,866,875,
				            883,891,899,906,914,920,927,934,940,946,951,956,961,966,970,974,978,982,985,988,990,993,995,996,998,999,999,1000};
const uint16_t cos1000[90]={1000,999,999,998,996,995,993,990,988,985,982,978,974,970,966,961,956,951,946,940,934,927,921,914,906,899,891,883,875,866,
		                    857,848,839,829,819,809,799,788,777,766,755,743,731,719,707,695,682,669,656,643,629,616,602,588,574,559,545,530,515,500,
							485,469,454,438,423,407,391,375,358,342,326,309,292,276,259,242,225,208,191,174,156,139,122,105,87,70,52,35,17,0};

// Показ одной точки, смена положения луча и добавление еще одной точки
// Вход угол текущая дистанция
void showOnePoint(uint16_t angle, uint16_t dist)
{
	uint32_t  x1,y1;

	if(dist>RADIUS) dist=RADIUS;                                    // Ограничить значения радиусом круга
	ST7735_DrawLine(CENTRE_X, CENTRE_Y,xLine, yLine, ST7735_BLACK);	// Стереть старую линию
	if(zPoint==1) ST7735_DrawPixel(xPoint, yPoint, ST7735_CYAN);    // Восстановить точку с учетом цвета
	else          ST7735_DrawPixel(xPoint, yPoint, ST7735_YELLOW);

   // Расчет новой конечной точки и точки на лидаре  В зависимости от квадранта угла
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


ST7735_DrawPixel(data[angle].x,data[angle].y, ST7735_BLACK); // Стереть старую точку
data[angle].x=xPoint; data[angle].y=yPoint;                  // Запомнить новую точку
if (dist==RADIUS) zPoint=1; else zPoint=0;                   // Цвет привышения дистанции голубой
ST7735_DrawLine(CENTRE_X, CENTRE_Y,x1, y1, ST7735_GREEN);	 // Новая линия
//ST7735_DrawPixel(xPoint, yPoint, ST7735_RED);

xLine=x1;
yLine=y1;
}

// Показ Шкалы  (храняться значения шкалы в дм, 0 не выводим)
const uint8_t scaleLavel[6][6] = {{ 0,0,5,0,0,10},    // шкала до метра
								  {0,0,10,0,0,20},    // шкала до 2 метра
								  {0,10,0,20,0,30},   // шкала до 3 метра
								  {0,0,20,0,0,40},    // шкала до 4 метра
								  {0,10,20,30,40,50}, // шкала до 5 метра
								  {0,20,0,40,0,60}    // шкала до 6 метра
								  };
void scale_show(void)
{
uint8_t i;
char buf[8];
	ST7735_DrawFastVLine(2*CENTRE_X+4,0, CENTRE_Y, ST7735_WHITE);
	for (i=0;i<6;i++){
		ST7735_DrawFastHLine(2*CENTRE_X+4,3+10*i, 4, ST7735_WHITE);
		if (scaleLavel[scale-1][i]>0){
			itoa(scaleLavel[scale-1][i]*10,buf,10);
			ST7735_DrawString(2*CENTRE_X+10, (5-i)*10,buf, Font_7x10, ST7735_RED, ST7735_BLACK);} else
			ST7735_DrawString(2*CENTRE_X+10, (5-i)*10,"   ", Font_7x10, ST7735_RED, ST7735_BLACK);
	}
	// Масштаб
	itoa(scale,buf,10);
	ST7735_DrawString(0, 0, "x", Font_7x10, ST7735_WHITE, ST7735_BLACK);
	ST7735_DrawString(7, 0, buf, Font_7x10, ST7735_WHITE, ST7735_BLACK);
	fScale=0; // Сбросить флаг необходимости перечерчиавания шкалы
}
#ifndef UART_DMA
void readOnePoket(void)
{
 	  int32_t diff;
	  int16_t angle_per_sample;
	  uint16_t i;
      uint32_t index;          // Угол в градусах, он же индекс массива данных
      typeHeader header;
      onePoint *point;         // Указатель на одно измерение приходящее с лидара
      uint8_t pressKey=0;      // Отпускание клавиши

      #ifdef AVERAGING       // Для усреднения
	   uint32_t sum=0,n=0;
	   uint32_t indexOld=0;
      #endif
	  state = START1;        // Начальная стадия
	   while (1)
	  {
	   if (HAL_GPIO_ReadPin(GPIOB, ENC_BTN_Pin)==0) pressKey=0;  // Клавиша отпущена
	   if (state == START1)   // Поиск заголовка из двух байт
	   {
		   HAL_UART_Receive(&huart2,(uint8_t*)rxBuf, 1, HAL_MAX_DELAY);
		   if (rxBuf[0]==0xAA) { state = START2; } else { continue; }
	   }
	   else if (state == START2)
	   {
		   HAL_UART_Receive(&huart2, (uint8_t*)rxBuf, 1, HAL_MAX_DELAY);
		   if (rxBuf[0]==0x55) { state = HEADER; } else { state = START1; continue;}
	   }
	   else if (state == HEADER) {  // Разбор заголовка посылки
		    HAL_UART_Receive(&huart2, (uint8_t*)&header, 8, HAL_MAX_DELAY);
			diff = header.stop_angle - header.start_angle;                           // Диапазон углов
			if (header.stop_angle<header.start_angle) diff=0xB400-header.start_angle+header.stop_angle; // если перешли через 0  (0xB400 скорее всего максимальный угол)
			angle_per_sample = 0;                                                    // угол одного образца
			if (diff > 1) angle_per_sample = diff / (header.data_lenght-1);          // вычисление изменения угла на одно измерение в посылке
			if (header.pack_type!= 0x38) { }                                         // не тот тип пакета/лидара
			state = DATA;
			continue;
	   }
	   else if (state == DATA) {                                                     // Чтение измерений в пакете
		   state = START1;
		   HAL_UART_Receive_IT(&huart2, (uint8_t*)rxBuf, header.data_lenght * 3);          // читаем все данные
		   osDelay(11);                                                             // Время должно быть больше времени приема данных 12 работает
		   HAL_GPIO_TogglePin(GPIOB, LED2_Pin);                                     // Инвертирование состояния светодиода

		   // При огруглени углов до градусов получается несколько точек с одним углом, пытаемся их усреднить (признак AVERAGING)
			#ifdef AVERAGING
			 sum=0,n=0;                                                            // Признак первой итерации
			#endif
		   for (i=0;i<header.data_lenght;i++){                                            // По всем измерениям пакета
	    	   index = (header.start_angle + angle_per_sample * i)*360/0xB400;            // расчет угла в градусах
	    	   if (index>359) index=index-359;                                            // переход через 0
	    	   if (index>359) index=359;
	    	   point=(onePoint*)&rxBuf[i*3];
               #ifdef AVERAGING    // накопление суммы
				   if (n==0) index=indexOld;                                       // Первая итерация
				   if (index==indexOld) { // новый угол равен старому - накопление суммы
					   sum=sum+point->distance;
					   n++;
				   }
				   else {                // запоминание и усреднение точки
					   data[indexOld].distance = sum/n;
					   data[indexOld].quality=point->quality;
					   sum=point->distance; // Добавить новое значение
					   n=1;
					   indexOld=index;
				   }
                #else   // без усреднения
					data[index].distance = point->distance;
					data[index].quality=point->quality;
                #endif

	       } // for
           #ifdef AVERAGING
	          if (sum>0) {data[index].distance = sum/n;data[index].quality=rxBuf[i*3 + 0];}// Последняя точка
           #endif
    	} // if
           // Чтение кнопки энкодера - изменение масштаба
	  		 if ((HAL_GPIO_ReadPin(GPIOB, ENC_BTN_Pin)==1)&&(pressKey==0)) {
	  			 osDelay(20);
	  			 if (HAL_GPIO_ReadPin(GPIOB, ENC_BTN_Pin)==1){
	  				if(scale<MAXZOOM) scale++; else scale=1;
	  				pressKey=1;
	  				fScale=1; // Надо перерисовать шкалу
	  			 }
	  		 }
	  }   // while
}
#endif

// Показать радар
void showRadar(void){
uint16_t i;
uint32_t time;
uint16_t dt=0;
char buf[8];
time=HAL_GetTick();
     for (i=0;i<360;i++){ // Показ графика
   			 showOnePoint(i,data[i].distance/(scale*CONST_SCALE));  // Масштабирование
   		     data[i].distance=0;                                    // Обнулить данные
   		     if(fScale==1) scale_show(); // было изменение масштаба
         }// for
// Показ времени полного круга (измерение)
dt=HAL_GetTick()-time;
itoa(dt,buf,10);
ST7735_DrawString(105, 118, buf, Font_7x10, ST7735_WHITE, ST7735_BLACK);
ST7735_DrawString(140, 118, "ms", Font_7x10, ST7735_WHITE, ST7735_BLACK);
}

// Показать стартовый экран, инициализация дисплея
void showStartScreen(void){
	   ST7735_Init();
	   ST7735_Backlight_On(); // Включить подсветку дисплея
	   ST7735_SetRotation(3);
	   ST7735_FillScreen(ST7735_BLACK);
	   ST7735_DrawImage(0, 10, 160, 58, (const uint16_t*)gImage_lidar); // вывести заставку
	   ST7735_DrawString(10, 75, "LIDAR MB-1R2T", Font_11x18, ST7735_YELLOW, ST7735_BLACK);
	  // ST7735_DrawFastHLine(10,50, 140, ST7735_YELLOW);
	   ST7735_DrawFastHLine(10,94, 140, ST7735_YELLOW);
	   ST7735_DrawString(0, 98, "Encoder button - zoom", Font_7x10, ST7735_WHITE, ST7735_BLACK);
	   ST7735_DrawString(0, 108, "Hardware version: 1.4", Font_7x10, ST7735_RED, ST7735_BLACK);
	   ST7735_DrawString(0, 118, "Software version:", Font_7x10, ST7735_RED, ST7735_BLACK);
	   ST7735_DrawString(127, 118, VERSION, Font_7x10, ST7735_RED, ST7735_BLACK);
	   HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);    // Установить светодиод 2 в 1
	   HAL_Delay(3000);
	   ST7735_FillScreen(ST7735_BLACK);
	   ST7735_DrawCircle(CENTRE_X, CENTRE_Y, RADIUS, ST7735_BLUE);
	   #ifdef UART_DMA  // Вывести режим работы
	   ST7735_DrawString(120, 106, " DMA", Font_7x10, ST7735_YELLOW, ST7735_BLACK);
	   #else
	   ST7735_DrawString(115, 106, "no DMA", Font_7x10, ST7735_YELLOW, ST7735_BLACK);
	   #endif
	   scale_show();
}
