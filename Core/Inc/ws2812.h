//written by Fred Xiao

#ifndef __WS2812_H__
#define __WS2812_H__
 
 
#include "main.h" 


#define WS2812_SPI_UNIT     hspi6
extern SPI_HandleTypeDef WS2812_SPI_UNIT;
 
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);
void RGB_Color_Ctrl(uint8_t r, uint8_t g, uint8_t b);


#define PIXEL_NUM 5

extern uint8_t WS2812_Buffer[PIXEL_NUM * 24];
void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b);
void WS2812_Test(void);


#endif
