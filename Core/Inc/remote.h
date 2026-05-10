//written by Fred Xiao

#ifndef __REMOTE_H
#define __REMOTE_H
#include "main.h"
#define RX_BUF_SIZE 256

// 遥控器数据结构体
typedef struct {
    float angle;           // 摇杆角度
    float distance;        // 摇杆推力百分比
    uint8_t btn_1;         // 前倾
    uint8_t btn_2;         // 后仰
    uint8_t btn_3;         // 击球
    uint8_t btn_4;         // 备用
    uint8_t update_flag;   // 更新标志位(可选)
} RemoteData_t;

extern RemoteData_t rc_data;

extern uint8_t rx_ready_flag;

// RX_BUF_SIZE在usart.h中统一定义
extern uint8_t usart10_rx_buf[RX_BUF_SIZE];


uint16_t Calculate_CRC16(const char *data, uint16_t len);
void Process_RC_Packet(char *packet, uint16_t len);

#endif
