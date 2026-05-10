//written by Fred Xiao


#ifndef __UNITREE_GO_H
#define __UNITREE_GO_H

#include "main.h"

// 确保结构体按1字节对齐，避免编译器优化对齐
#pragma pack(push, 1)
/**
 * @brief 电机模式控制信息
 */
typedef struct
{
    uint8_t id : 4;      // 电机ID: 0,1...,13,14 15表示向所有电机广播数据(此时无返回)
    uint8_t status : 3;  // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    uint8_t reserve : 1; // 保留位
} RIS_Mode_t;            // 控制模式 1Byte

/**
 * @brief 电机状态控制信息
 */
typedef struct
{
    int16_t tor_des; // 期望关节输出扭矩 unit: N.m      (q8)
    int16_t spd_des; // 期望关节输出速度 unit: rad/s    (q8)
    int32_t pos_des; // 期望关节输出位置 unit: rad      (q15)
    int16_t k_pos;   // 期望关节刚度系数 unit: -1.0-1.0 (q15)
    int16_t k_spd;   // 期望关节刚度系数 unit: -1.0-1.0 (q15)
} RIS_Comd_t;         // 控制参数 12Byte

/**
 * @brief 电机状态反馈信息
 */
typedef struct
{
    int16_t torque;      // 实际关节输出扭矩 unit: N.m     (q8)
    int16_t speed;       // 实际关节输出速度 unit: rad/s   (q8)
    int32_t pos;         // 实际关节输出位置 unit: rad     (q15)
    int8_t temp;         // 电机温度: -128~127°C
    uint8_t MError : 3;  // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障 5-7.保留
    uint16_t force : 12; // 足端气压传感器数据 12bit (0-4095)
    uint8_t none : 1;    // 保留位
} RIS_Fbk_t;             // 状态数据 11Byte

/**
 * @brief 控制数据包格式
 */
typedef struct
{
    uint8_t head[2]; // 包头         2Byte
    RIS_Mode_t mode; // 电机控制模式  1Byte
    RIS_Comd_t comd; // 电机期望数据 12Byte
    uint16_t CRC16;  // CRC          2Byte
} RIS_ControlData_t;  // 主机控制命令     17Byte

/**
 * @brief 电机反馈数据包格式
 */
typedef struct
{
    uint8_t head[2]; // 包头         2Byte
    RIS_Mode_t mode; // 电机控制模式  1Byte
    RIS_Fbk_t fbk;   // 电机反馈数据 11Byte
    uint16_t CRC16;  // CRC          2Byte
    // uint8_t data[16];
} RIS_MotorData_t;   // 电机返回数据     16Byte
#pragma pack(pop)

/// @brief 电机指令结构体
typedef struct
{
    unsigned short id;   // 电机ID，15代表广播数据包
    unsigned short mode; // 0:空闲 1:FOC控制 2:电机标定
    float T;             // 期望关节的输出力矩(电机本身的力矩)(Nm)
    float W;             // 期望关节速度(电机本身的速度)(rad/s)
    float Pos;           // 期望关节位置(rad)
    float K_P;           // 关节刚度系数(0-25.599)
    float K_W;           // 关节速度系数(0-25.599)

    RIS_ControlData_t motor_send_data; // 发送数据包

} UnitreeMotorCmd_t;

/// @brief 电机反馈结构体
typedef struct
{
    unsigned char motor_id; // 电机ID
    unsigned char mode;     // 0:空闲 1:FOC控制 2:电机标定
    int Temp;               // 温度
    int MError;             // 错误码
    float T;                // 当前实际电机输出力矩(电机本身的力矩)(Nm)
    float W;                // 当前实际电机速度(电机本身的速度)(rad/s)
    float Pos;              // 当前电机位置(rad)
    float pos;              // 当前电机相对初始位置(rad)
    int correct;            // 接收数据是否完整(1完整，0不完整)
    int footForce;          // 足端力传感器原始数值

    uint16_t calc_crc;      // 计算得到的CRC
    uint32_t timeout;       // 通讯超时计数
    uint32_t bad_msg;       // CRC校验错误计数
    RIS_MotorData_t motor_recv_data; // 接收数据包

} UnitreeMotorData_t;

// 电机数量配置
#define UNITREE_MOTOR_NUM 8

// 全局电机命令和反馈数组
extern UnitreeMotorCmd_t unitree_cmd[UNITREE_MOTOR_NUM];
extern UnitreeMotorData_t unitree_data[UNITREE_MOTOR_NUM];

extern int error_unitree;
extern float unitree_angle_init[UNITREE_MOTOR_NUM] ;
extern int unitree_flag[UNITREE_MOTOR_NUM];

// 函数声明
void unitree_modify_data(UnitreeMotorCmd_t *motor_s);
void unitree_extract_data(UnitreeMotorData_t *motor_r);
void unitree_cmd_create(UnitreeMotorCmd_t *cmd, uint8_t id, uint8_t mode, 
                        float K_P, float K_W, float Pos, float W, float T);
HAL_StatusTypeDef unitree_send_command(uint8_t motor_id);
HAL_StatusTypeDef unitree_receive_feedback(uint8_t motor_id);
HAL_StatusTypeDef unitree_communicate(uint8_t motor_id);
void unitree_init(void);
uint16_t unitree_crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len);

// 调试函数
void unitree_print_motor_data(uint8_t motor_id);

typedef struct
{
    uint8_t data[30];
}DataRecive_t;
extern DataRecive_t unitree_data_receive[UNITREE_MOTOR_NUM];

extern int unitree_pos[2];

#endif /* __UNITREE_GO_H */