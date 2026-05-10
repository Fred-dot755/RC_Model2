//written by Fred Xiao

#include "main.h"

#define RX_BUF_SIZE 256
RemoteData_t rc_data;

// 数据解析标志位
uint8_t rx_ready_flag = 0;
uint8_t usart10_rx_buf[RX_BUF_SIZE] __attribute__((aligned(32)));


// —— CRC-16校验计算 (与JS端完全一致) ——
uint16_t Calculate_CRC16(const char *data, uint16_t len) 
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint8_t)data[i];
        for (uint8_t j = 0; j < 8; j++) {
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

void Process_RC_Packet(char *packet, uint16_t len) 
{
    // 1. 寻找帧头和帧尾
    char *start = strchr(packet, '[');
    if (start == NULL) return; 

    char *end = strchr(start, ']');
    if (end == NULL) return;   
    
    uint16_t data_len = end - start + 1;

    // 确保 ']' 后面至少有 4 个字符作为 CRC
    if (strlen(end) < 5) return; 

    // 2. 计算本地 CRC (如果你想进一步优化，可以把 Calculate_CRC16 换成查表法)
    uint16_t calc_crc = Calculate_CRC16(start, data_len);

    // 3. 【优化核心】将接收到的 4 位 HEX 字符串快速转换为 uint16_t 数字
    uint16_t rx_crc = 0;
    for (int i = 1; i <= 4; i++) {
        rx_crc <<= 4; // 左移 4 位腾出空间
        char c = end[i];
        if (c >= '0' && c <= '9') {
            rx_crc |= (c - '0');
        } else if (c >= 'A' && c <= 'F') {
            rx_crc |= (c - 'A' + 10);
        } else if (c >= 'a' && c <= 'f') { // 兼容小写
            rx_crc |= (c - 'a' + 10);
        } else {
            return; // 遇到非十六进制字符，说明数据错乱，丢弃
        }
    }

    // 4. 数字直接比对，极速！
    if (calc_crc == rx_crc) 
    {
        // 5. 【优化核心】摒弃 sscanf，使用 strtof / strtol 配合指针快速滑动解析
        // 格式假设完美对应: [angle;distance;b1;b2;b3;b4]
        char *p = start + 1; // 指针跳过 '['
        
        rc_data.angle    = strtof(p, &p); p++; // 解析完浮点后，p会自动指向';'，p++跳过';'
        rc_data.distance = strtof(p, &p); p++;
        rc_data.btn_1    = (uint8_t)strtol(p, &p, 10); p++;
        rc_data.btn_2    = (uint8_t)strtol(p, &p, 10); p++;
        rc_data.btn_3    = (uint8_t)strtol(p, &p, 10); p++;
        rc_data.btn_4    = (uint8_t)strtol(p, &p, 10);

        // 防止前端传来异常的非法数据
        if(rc_data.distance > 1.0f) rc_data.distance = 1.0f;
        if(rc_data.distance < 0.0f) rc_data.distance = 0.0f;
        
        // 更新标志位
        rc_data.update_flag = 1; 
    }
}