/* USER CODE BEGIN Header */
//written by Fred Xiao

/**
  ******************************************************************************
  * @file    motordm.h
  * @brief   Header for DM (达妙) motor control
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Robocon Team.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTORDM_H__
#define __MOTORDM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

// 达妙电机常量定义
#define DM_MOTOR_CAN_ID        0x01   // 电机CAN ID（可通过调试助手设置）
#define DM_FEEDBACK_ID         0x00   // 默认反馈ID，若修改需同步调整

#define DM_PI 3.1415926535f

// 物理量限幅（需与达妙调试助手中的设置一致，以下为常见默认值）
#define DM_P_MIN  -12.5f
#define DM_P_MAX   12.5f
#define DM_V_MIN  -45.0f
#define DM_V_MAX   45.0f
#define DM_T_MIN  -40.0f  // 说明书显示峰值扭矩为40NM
#define DM_T_MAX   40.0f

/* USER CODE END Private defines */

/* USER CODE BEGIN Exported types */
// 达妙电机反馈数据结构
typedef struct {
    uint8_t id;
    uint8_t error;
    
    // 原始数据
    uint16_t pos_raw;
    uint16_t vel_raw;
    uint16_t tor_raw;

    // 零点初始化
    float pos_offset_rad;   // 上电时的绝对物理位置
    uint8_t is_initialized; // 是否已捕获初始位置

    // 最终输出物理量
    float position_deg;     // 相对角度 (以当前位置为0)
    float position_rad;     // 相对弧度
    float velocity_rads;    // 当前速度
    float torque_Nm;        // 当前扭矩
} DM4310_Feedback_t;

/* USER CODE END Exported types */

/* USER CODE BEGIN Exported variables */
extern DM4310_Feedback_t dm4310_fb[2];

/* USER CODE END Exported variables */

/* USER CODE BEGIN Exported functions prototypes */

// 初始化函数
void DM_Motor_Init(void);

// 发送控制函数
void DM_CAN_Send_PosVel_Mode(float p_des, float v_des, uint8_t CAN_ID);
void DM_CAN_Send_Vel_Mode(float v_des, uint8_t CAN_ID);
void DM_CAN_Enable_Motor(uint8_t CAN_ID);
void DM_CAN_Disable_Motor(uint8_t CAN_ID) ;
void DM_CAN_Save_Zero_Motor(uint8_t CAN_ID) ;


// 反馈处理函数（需要在FDCAN回调中调用）
void DM_Process_Rx_Message(uint32_t StdId, uint8_t* data);

// 获取反馈数据
float DM_Get_Position_Deg(void);
float DM_Get_Position_Rad(void);
float DM_Get_Velocity_RadS(void);
float DM_Get_Torque_Nm(void);
uint8_t DM_Get_Error_Status(void);

/* USER CODE END Exported functions prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __MOTORDM_H__ */