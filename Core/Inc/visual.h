#ifndef VISUAL_H
#define VISUAL_H

#include "main.h"


#pragma pack(push, 1)

// enum : uint8_t
// {
//     DIR_NONE  = 0,
//     DIR_LEFT  = 1,
//     DIR_RIGHT = 2,
//     DIR_FRONT = 3,
//     DIR_BACK  = 4,
//     DIR_UP    = 5,
//     DIR_DOWN  = 6
// };

typedef struct
{
    uint8_t cell;        // 点位：1~12，0无效
    uint8_t horizontal_s;    // 左1 右2 前3 后4，0无
    uint8_t vertical_s;  // 上5 下6，0无
} __attribute__((packed)) MeilinPointCmd;


typedef struct {
     uint8_t head[4];
    int workl_mode;
    int bool_graping;
    int bool_getKFS;
    float xyz_in_base[3];
    //float wepend_offset_x;
    float x_map;
    float y_map;
    int target_cell;
    //float yaw_map;
    int i;
    uint8_t meilin_count;               // 有效点数量，最多9
    MeilinPointCmd meilin_points[9];   // 不足9个填{0,0,0}
    uint16_t crc16;  // CRC16校验值（用于数据完整性校验）
    
} __attribute__((packed)) ReceiveData_t;

typedef struct {
    uint8_t head[4];         // 'M', 'E', 'I', 'C'
    int work_mode;
    int yaw;
    int pitch;
    uint16_t crc16;
} __attribute__((packed)) SendData_t;

#pragma pack(pop)

#define MEIC_PACKET_SIZE sizeof(ReceiveData_t)

uint16_t get_crc16(const uint8_t *data, uint32_t len);

bool check_crc16(const uint8_t *data, uint32_t len);

bool meic_protocol_parse_packet(const uint8_t *packet, uint16_t len, ReceiveData_t *out_data);

extern ReceiveData_t visual_data;
extern SendData_t visual_send_data;

void meic_protocol_send_packet_dma(UART_HandleTypeDef *huart, int pitch ,int yaw);

#endif
