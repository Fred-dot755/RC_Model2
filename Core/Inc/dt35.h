#ifndef __DT35_H
#define __DT35_H

#include "main.h"

// 激光数据结构体
typedef struct 
{
    uint16_t dist[4];     // 4个通道的距离数据 (mm)
    uint8_t update_flag;  // 数据更新标志位
} DT35_Data_t;

extern DT35_Data_t dt35_data;
// 用于替换原来的 usart1_rx_buf
#define DT35_RX_BUF_SIZE 20
extern uint8_t dt35_rx_buf[DT35_RX_BUF_SIZE]; 

void DT35_Data_Decode(uint8_t *buf, uint16_t len);
uint16_t DT35_Calculate_CRC16(uint8_t *data, uint16_t length);

#endif
