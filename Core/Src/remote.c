#include "main.h"

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


// 处理一帧接收到的数据
void Process_RC_Packet(char *packet, uint16_t len) 
{
    // 1. 寻找帧头 '['
    char *start = strchr(packet, '[');
    if (start == NULL) return; // 没找到头，直接丢弃

    // 2. 【关键修复】从帧头所在的位置开始，向后寻找帧尾 ']'
    char *end = strchr(start, ']');
    if (end == NULL) return;   // 没找到尾巴，说明数据不完整，直接丢弃
    
    // (因为是从 start 向后找的，所以 end 必然大于 start，不再需要判断 end < start)

    // 3. 截取 dataPart 的长度 (包含 '[' 和 ']')
    uint16_t data_len = end - start + 1;

    // 4. 确保 ']' 后面至少有 4 个字符作为 CRC
    if (strlen(end) < 5) return; // 修复越界隐患：确保字符串末尾有足够的长度供 CRC 读取

    // 5. 计算本地 CRC
    uint16_t calc_crc = Calculate_CRC16(start, data_len);

    // JS代码逻辑是 hi在前, lo在后，并转为大写 HEX
    char calc_crc_str[5];
    sprintf(calc_crc_str, "%02X%02X", (calc_crc >> 8) & 0xFF, calc_crc & 0xFF);

    // 5. 提取接收到的 CRC (紧跟在 ']' 后面的4个字符)
    char rx_crc_str[5] = {0};
    strncpy(rx_crc_str, end + 1, 4);

    // 6. 比较 CRC
    if (strncmp(rx_crc_str, calc_crc_str, 4) == 0) 
    {
        // 定义 4 个标准 int 来做缓冲
        int b1 = 0, b2 = 0, b3 = 0, b4 = 0;
        
        // 校验通过，提取数据
        // 使用 %f 和 %d 分别对应 float 和 uint8_t
        int parsed = sscanf(start, "[%f;%f;%d;%d;%d;%d]", 
                            &rc_data.angle, 
                            &rc_data.distance, 
                            &b1, &b2, &b3, &b4);
        
        // 如果成功解析了 6 个参数，说明包格式完美                 
        // if(parsed == 6) {
            // 解析成功后，再安全地写入 uint8_t 变量
            rc_data.btn_1 = (uint8_t)b1;
            rc_data.btn_2 = (uint8_t)b2;
            rc_data.btn_3 = (uint8_t)b3;
            rc_data.btn_4 = (uint8_t)b4;
            
            // 仅仅做数据更新
            rc_data.update_flag = 1; 
            
            // 防止前端传来异常的非法数据
            if(rc_data.distance > 1.0f) rc_data.distance = 1.0f;
            if(rc_data.distance < 0.0f) rc_data.distance = 0.0f;
        // } 
    }
}

