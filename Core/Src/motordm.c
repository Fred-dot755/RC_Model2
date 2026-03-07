/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motordm.c
  * @brief   DM (达妙) motor control implementation
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Robocon Team.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "motordm.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
// 线性映射辅助函数（达妙标准映射）
static float dm_uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}

/* USER CODE END Private defines */

/* USER CODE BEGIN Private variables */
DM4310_Feedback_t dm4310_fb = {0};

/* USER CODE END Private variables */

/* USER CODE BEGIN Exported functions */

/**
  * @brief  达妙电机初始化
  * @retval None
  */
void DM_Motor_Init(void)
{
    dm4310_fb.id = 0;
    dm4310_fb.error = 0;
    dm4310_fb.pos_raw = 0;
    dm4310_fb.vel_raw = 0;
    dm4310_fb.tor_raw = 0;
    dm4310_fb.pos_offset_rad = 0.0f;
    dm4310_fb.is_initialized = 0;
    dm4310_fb.position_deg = 0.0f;
    dm4310_fb.position_rad = 0.0f;
    dm4310_fb.velocity_rads = 0.0f;
    dm4310_fb.torque_Nm = 0.0f;
}

/**
  * @brief  处理接收到的CAN消息
  * @param  StdId: 标准ID
  * @param  data: 数据数组
  * @retval None
  */
void DM_Process_Rx_Message(uint32_t StdId, uint8_t* data)
{
    if (StdId == 0x03 ) 
    {
        dm4310_fb.id = data[0];
        dm4310_fb.error = (data[1] >> 4) & 0x0F;


        uint16_t p_int = (uint16_t)((data[2] << 8) | data[3]);
        
        uint16_t v_int = (uint16_t)(((data[4] & 0x0F) << 8) | data[5]);
        
        uint16_t t_int = (uint16_t)(((data[6] & 0x0F) << 8) | data[7]);

        float current_abs_rad = dm_uint_to_float(p_int, DM_P_MIN, DM_P_MAX, 16);

        if (dm4310_fb.is_initialized == 0) {
            dm4310_fb.pos_offset_rad = current_abs_rad; // 记录上电时的绝对位置
            dm4310_fb.is_initialized = 1;
        }

        dm4310_fb.position_rad = current_abs_rad - dm4310_fb.pos_offset_rad;
        dm4310_fb.position_deg = dm4310_fb.position_rad * (180.0f / DM_PI);
        
        dm4310_fb.velocity_rads = dm_uint_to_float(v_int, DM_V_MIN, DM_V_MAX, 12);
        dm4310_fb.torque_Nm     = dm_uint_to_float(t_int, DM_T_MIN, DM_T_MAX, 12);
    }
}

/**
  * @brief  发送位置速度模式控制命令
  * @param  p_des_deg: 期望位置（度）
  * @param  v_des_deg: 期望速度（度/秒）
  * @param  CAN_ID: CAN ID
  * @retval None
  */
void DM_CAN_Send_PosVel_Mode(float p_des_deg, float v_des_deg, uint8_t CAN_ID)
{
    float p_target_abs = (p_des_deg * (DM_PI / 180.0f)) + dm4310_fb.pos_offset_rad;
    float v_limit_rad = v_des_deg * (DM_PI / 180.0f);

    FDCAN_TxHeaderTypeDef header;
    uint8_t data[8];
    
    header.Identifier = 0x100 + CAN_ID;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0;
    
    float *p_ptr = (float *)&data[0];
    float *v_ptr = (float *)&data[4];
    *p_ptr = p_target_abs;
    *v_ptr = v_limit_rad;
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &header, data);
}

/**
  * @brief  发送速度模式控制命令
  * @param  v_des_deg: 期望速度（度/秒）
  * @param  CAN_ID: CAN ID
  * @retval None
  */
void DM_CAN_Send_Vel_Mode(float v_des_deg, uint8_t CAN_ID)
{
    float v_limit_rad = v_des_deg * (DM_PI / 180.0f);

    FDCAN_TxHeaderTypeDef header;
    uint8_t data[8] = {0};
    
    header.Identifier = 0x100 + CAN_ID;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0;
    
    // 速度模式：发送速度值
    float *v_ptr = (float *)&data[0];
    *v_ptr = v_limit_rad;
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &header, data);
}

/**
  * @brief  使能电机
  * @param  CAN_ID: CAN ID
  * @retval None
  */
void DM_CAN_Enable_Motor(uint8_t CAN_ID) 
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

    header.Identifier = 0x100 + CAN_ID;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &header, data);
}

/**
  * @brief  获取当前位置（度）
  * @retval 当前位置（度）
  */
float DM_Get_Position_Deg(void)
{
    return dm4310_fb.position_deg;
}

/**
  * @brief  获取当前位置（弧度）
  * @retval 当前位置（弧度）
  */
float DM_Get_Position_Rad(void)
{
    return dm4310_fb.position_rad;
}

/**
  * @brief  获取当前速度（弧度/秒）
  * @retval 当前速度（弧度/秒）
  */
float DM_Get_Velocity_RadS(void)
{
    return dm4310_fb.velocity_rads;
}

/**
  * @brief  获取当前扭矩（Nm）
  * @retval 当前扭矩（Nm）
  */
float DM_Get_Torque_Nm(void)
{
    return dm4310_fb.torque_Nm;
}

/**
  * @brief  获取错误状态
  * @retval 错误状态
  */
uint8_t DM_Get_Error_Status(void)
{
    return dm4310_fb.error;
}

/* USER CODE END Exported functions */