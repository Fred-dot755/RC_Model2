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
    uint8_t has_true_kfs;  // 1 有真KFS，需要抓；0 没有
} __attribute__((packed)) MeilinPointCmd;


typedef struct {
     uint8_t head[4];
    int workl_mode;
    int bool_graping;
    int bool_getKFS;
    float xyz_in_base[3];
    float kfs_x;///取矿偏移
    float kfs_y;//取矿偏移
    float x_map;//雷达
    float y_map;//雷达
    int target_cell;
    int targetid;
    int dx;//对武器偏移
    //float yaw_map;
    int i;
    uint8_t meilin_count;               // 有效点数量，最多10
    MeilinPointCmd meilin_points[10];   // 不足10个填{0,0,0}
    int hmi_start;
    int hmi_color;
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
#define VISUAL_RADAR_TIMEOUT_MS 500U
#define VISUAL_RADAR_MAX_STEP 2.0f

uint16_t get_crc16(const uint8_t *data, uint32_t len);

bool check_crc16(const uint8_t *data, uint32_t len);

bool meic_protocol_parse_packet(const uint8_t *packet, uint16_t len, ReceiveData_t *out_data);

extern ReceiveData_t visual_data;
extern SendData_t visual_send_data;

void meic_protocol_send_packet_dma(UART_HandleTypeDef *huart, int pitch ,int yaw ,int work_mode);
void visual_radar_on_packet(void);
bool visual_radar_is_safe(void);
void visual_radar_clear_fault(void);

#endif
