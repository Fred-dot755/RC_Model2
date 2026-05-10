#ifndef VISUAL_H
#define VISUAL_H

#include "main.h"


#pragma pack(push, 1)

typedef struct {
    uint8_t head[4];         // 'M', 'E', 'I', 'C'
    int work_mode;       // int 在 32 位系统上是 4 字节
    int bool_graping;
    int bool_getKFS;
    float xyz_in_base[3];    // 3 * 4 = 12 字节
    float x;
    float y;
    int i;
    uint16_t crc16;          // 2 字节
} __attribute__((packed)) ReceiveData_t;

typedef struct {
    uint8_t head[4];         // 'M', 'E', 'I', 'C'
    int work_mode;
    int bool_graping;
    uint16_t crc16;
} __attribute__((packed)) SendData_t;

#pragma pack(pop)

#define MEIC_PACKET_SIZE sizeof(ReceiveData_t)

uint16_t get_crc16(const uint8_t *data, uint32_t len);

bool check_crc16(const uint8_t *data, uint32_t len);

bool meic_protocol_parse_packet(const uint8_t *packet, uint16_t len, ReceiveData_t *out_data);

extern ReceiveData_t visual_data;
extern SendData_t visual_send_data;

void meic_protocol_send_packet_dma(UART_HandleTypeDef *huart, int work_mode, int bool_graping);

#endif
