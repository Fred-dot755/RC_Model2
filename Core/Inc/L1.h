//written by Fred Xiao

#ifndef __L1_H
#define __L1_H
#include "main.h"

// 测量模式定义
typedef enum {
    L1_MODE_IDLE = 0,
    L1_MODE_SINGLE,   // 单次测量
    L1_MODE_CONT,     // 连续测量
    L1_MODE_FAST      // 快速连续测量
} L1_MeasureMode_t;

typedef struct {
    uint32_t distance_mm; // 距离值
    uint8_t  error_code;  // 故障码
    uint8_t  is_updated;  // 更新标志
    L1_MeasureMode_t current_mode;
    UART_HandleTypeDef *huart; // UART句柄
    uint8_t tx_buf[32];        // 独立发送缓冲区
} L1_Data_t;

extern L1_Data_t L1_Sensor1;  // UART8
extern L1_Data_t L1_Sensor2;  // UART1

/* 核心控制接口 */
void L1_Init(L1_Data_t *sensor, UART_HandleTypeDef *huart);
void L1_Control(L1_Data_t *sensor, L1_MeasureMode_t mode); // 启动/切换测量模式
void L1_Stop(L1_Data_t *sensor);                     // 停止测量
void L1_SetLaser(L1_Data_t *sensor, uint8_t enable);       // 激光开关 (ASCII协议)
void L1_SetOffset(L1_Data_t *sensor, int16_t offset_mm);   // 设置偏移量 (ASCII协议)

/* 协议解析 */
void L1_Protocol_Parse(L1_Data_t *sensor, uint8_t *buf, uint16_t len);

#endif
