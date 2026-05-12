//written by Fred Xiao

#include "L1.h"
#include <string.h>
#include <stdio.h>

L1_Data_t L1_Sensor1 = {0};  // UART8
L1_Data_t L1_Sensor2 = {0};  // UART1

/**
 * @brief 计算 BCC 异或校验
 * 规则：从帧头到数据的所有字节进行异或
 */
static uint8_t L1_CalcBCC(uint8_t *dat, uint16_t len) {
    uint8_t bcc = 0;
    for (uint16_t i = 0; i < len; i++) bcc ^= dat[i];
    return bcc;
}

/**
 * @brief 核心发送函数：包含 H7 Cache 刷新逻辑
 */
static void L1_UART_Transmit_DMA_Safe(L1_Data_t *sensor, uint8_t *pData, uint16_t Size) {
    // 关键：将 Cache 里的新数据强制刷新到 SRAM，否则 DMA 发出的校验位是错的
    SCB_CleanDCache_by_Addr((uint32_t*)pData, Size);
    HAL_UART_Transmit_DMA(sensor->huart, pData, Size);
}

void L1_Init(L1_Data_t *sensor, UART_HandleTypeDef *huart) {
    memset(sensor, 0, sizeof(L1_Data_t));
    sensor->huart = huart;
}

/**
 * @brief 发送 HEX 控制命令 (单次/连续/快速)
 */
void L1_Control(L1_Data_t *sensor, L1_MeasureMode_t mode) {
    sensor->tx_buf[0] = 0xA5; // 帧头1
    sensor->tx_buf[1] = 0x5A; // 帧头2
    sensor->tx_buf[3] = 0x00; // 备用参数
    
    switch(mode) {
        case L1_MODE_SINGLE: sensor->tx_buf[2] = 0x02; break; // 单次测量功能码
        case L1_MODE_CONT:   sensor->tx_buf[2] = 0x03; break; // 连续测量功能码
        case L1_MODE_FAST:   sensor->tx_buf[2] = 0x04; break; // 快速连续测量功能码
        default: return;
    }
    
    sensor->tx_buf[4] = L1_CalcBCC(sensor->tx_buf, 4); // 计算异或校验
    sensor->current_mode = mode;
    L1_UART_Transmit_DMA_Safe(sensor, sensor->tx_buf, 5);
}

/**
 * @brief 停止测量指令
 */
void L1_Stop(L1_Data_t *sensor) {
    sensor->tx_buf[0] = 0xA5;
    sensor->tx_buf[1] = 0x5A;
    sensor->tx_buf[2] = 0x05; // 停止测量功能码
    sensor->tx_buf[3] = 0x00;
    sensor->tx_buf[4] = 0xFA; // 固定校验位 (A5^5A^05^00)
    
    L1_UART_Transmit_DMA_Safe(sensor, sensor->tx_buf, 5);
    sensor->current_mode = L1_MODE_IDLE;
}

/**
 * @brief 设置激光开关 (需使用 ASCII 协议)
 */
void L1_SetLaser(L1_Data_t *sensor, uint8_t enable) {
    // ASCII 指令：iLD:1 为开启，iLD:0 为关闭
    int len = sprintf((char*)sensor->tx_buf, "iLD:%d\r\n", enable ? 1 : 0); 
    L1_UART_Transmit_DMA_Safe(sensor, sensor->tx_buf, len);
}

/**
 * @brief 设置距离偏移量
 */
void L1_SetOffset(L1_Data_t *sensor, int16_t offset_mm) {
    // 偏移量范围 -10000~10000 毫米
    int len = sprintf((char*)sensor->tx_buf, "iSET:1,%d\r\n", offset_mm);
    L1_UART_Transmit_DMA_Safe(sensor, sensor->tx_buf, len);
}

/**
 * @brief HEX 响应解析逻辑
 */
void L1_Protocol_Parse(L1_Data_t *sensor, uint8_t *buf, uint16_t len) {
    if (len < 8) return;

    for (uint16_t i = 0; i <= len - 8; i++) {
        // 匹配 HEX 响应帧头 B4 69
        if (buf[i] == 0xB4 && buf[i+1] == 0x69) {
            // 验证 8 字节响应帧的校验位
            if (buf[i+7] == L1_CalcBCC(&buf[i], 7)) {
                if (buf[i+2] & 0x80) { // 出错响应 (功能码最高位为1)
                    sensor->error_code = buf[i+6]; // 获取具体故障码
                    sensor->distance_mm = 0;
                } else { // 正常响应
                    // 4字节距离大端解析 (高位在前)
                    sensor->distance_mm = (uint32_t)((buf[i+3] << 24) | (buf[i+4] << 16) | 
                                                     (buf[i+5] << 8)  | buf[i+6]);
                    sensor->error_code = 0;
                    sensor->is_updated = 1;
                }
            }
        }
    }
}
