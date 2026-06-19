#include "dt35.h"

DT35_Data_t dt35_data = {0};
uint8_t dt35_rx_buf[DT35_RX_BUF_SIZE];

// =========================================================================
// CRC16 校验 (与转接板一致)
// =========================================================================
uint16_t DT35_Calculate_CRC16(uint8_t *data, uint16_t length) 
{
    uint16_t crc = 0xFFFF; 
    uint16_t i, j;
    
    for (i = 0; i < length; i++) 
    {
        crc ^= data[i]; 
        for (j = 0; j < 8; j++) 
        {
            if (crc & 0x0001) {
                crc >>= 1;    
                crc ^= 0xA001; 
            } else {
                crc >>= 1;    
            }
        }
    }
    return crc;
}

// =========================================================================
// DT35 串口帧解析
// 数据包格式: AA 55 08 [D0_H D0_L] [D1_H D1_L] [D2_H D2_L] [D3_H D3_L] [CRC_H CRC_L] 0D 0A
// =========================================================================
void DT35_Data_Decode(uint8_t *buf, uint16_t len) 
{
    // 最短有效包长度为 15
    if (len < 15) return;

    // 滑动窗口寻找帧头
    for (uint16_t i = 0; i <= len - 15; i++) 
    {
        if (buf[i] == 0xAA && buf[i+1] == 0x55 && buf[i+2] == 0x08) 
        {
            // 校验 CRC (校验范围是前11个字节，即索引 0~10)
            uint16_t calc_crc = DT35_Calculate_CRC16(&buf[i], 11);
            uint16_t rx_crc = (buf[i+11] << 8) | buf[i+12];

            // 检查 CRC 与帧尾
            if (calc_crc == rx_crc && buf[i+13] == 0x0D && buf[i+14] == 0x0A) 
            {
                // 大端模式解析
                dt35_data.dist[0] = (buf[i+3] << 8) | buf[i+4];  // 通道 1 (D0)
                dt35_data.dist[1] = (buf[i+5] << 8) | buf[i+6];  // 通道 2 (D1)
                dt35_data.dist[2] = (buf[i+7] << 8) | buf[i+8];  // 通道 3 (D2)
                dt35_data.dist[3] = (buf[i+9] << 8) | buf[i+10]; // 通道 4 (D3)
                dt35_data.update_flag = 1;
                break; // 成功解析一帧即可跳出
            }
        }
    }
}
