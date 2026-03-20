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

static int dm_float_to_uint(float x_float, float x_min, float x_max, int bits) {
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/* USER CODE END Private defines */

/* USER CODE BEGIN Private variables */
DM4310_Feedback_t dm4310_fb[2] = {0};

/* USER CODE END Private variables */

/* USER CODE BEGIN Exported functions */

/**
  * @brief  达妙电机初始化
  * @retval None
  */
void DM_Motor_Init(void)
{
    dm4310_fb[0].id = 0;
    dm4310_fb[0].error = 0;
    dm4310_fb[0].pos_raw = 0;
    dm4310_fb[0].vel_raw = 0;
    dm4310_fb[0].tor_raw = 0;
    dm4310_fb[0].pos_offset_rad = 0.0f;
    dm4310_fb[0].is_initialized = 0;
    dm4310_fb[0].position_deg = 0.0f;
    dm4310_fb[0].position_rad = 0.0f;
    dm4310_fb[0].velocity_rads = 0.0f;
    dm4310_fb[0].torque_Nm = 0.0f;

    dm4310_fb[1].id = 0;
    dm4310_fb[1].error = 0;
    dm4310_fb[1].pos_raw = 0;
    dm4310_fb[1].vel_raw = 0;
    dm4310_fb[1].tor_raw = 0;
    dm4310_fb[1].pos_offset_rad = 0.0f;
    dm4310_fb[1].is_initialized = 0;
    dm4310_fb[1].position_deg = 0.0f;
    dm4310_fb[1].position_rad = 0.0f;
    dm4310_fb[1].velocity_rads = 0.0f;
    dm4310_fb[1].torque_Nm = 0.0f;

    DM_CAN_Enable_Motor(2);
    HAL_Delay(100);
    DM_CAN_Enable_Motor(3);
    HAL_Delay(100);
    DM_CAN_Save_Zero_Motor(2);
    HAL_Delay(100);
    DM_CAN_Save_Zero_Motor(3);
    HAL_Delay(100);
}

/**
  * @brief  处理接收到的CAN消息
  * @param  StdId: 标准ID
  * @param  data: 数据数组
  * @retval None
  */
void DM_Process_Rx_Message(uint32_t StdId, uint8_t* data)
{
  if (StdId == 0x02 ) 
    {
      dm4310_fb[0].id    = data[0] & 0x0F;
      dm4310_fb[0].error = (data[0] >> 4) & 0x0F;

      // POS: 16位，D[1]高字节，D[2]低字节
      uint16_t p_int = (uint16_t)((data[1] << 8) | data[2]);

      // VEL: 12位，D[3]全8位为高8位，D[4]高4位为低4位
      uint16_t v_int = (uint16_t)((data[3] << 4) | (data[4] >> 4));

      // TOR: 12位，D[4]低4位为高4位，D[5]全8位为低8位
      uint16_t t_int = (uint16_t)(((data[4] & 0x0F) << 8) | data[5]);

      float current_abs_rad = dm_uint_to_float(p_int, DM_P_MIN, DM_P_MAX, 16);

      if (dm4310_fb[0].is_initialized == 0) {
          dm4310_fb[0].pos_offset_rad = current_abs_rad;
          dm4310_fb[0].is_initialized = 1;
      }
      

      dm4310_fb[0].position_rad  = current_abs_rad - dm4310_fb[0].pos_offset_rad;//角度???
      dm4310_fb[0].position_deg  = (dm4310_fb[0].position_rad - current_abs_rad) * (180.0f / DM_PI);
      dm4310_fb[0].velocity_rads = dm_uint_to_float(v_int, DM_V_MIN, DM_V_MAX, 12);
      dm4310_fb[0].torque_Nm     = dm_uint_to_float(t_int, DM_T_MIN, DM_T_MAX, 12);
      }
    if (StdId == 0x03 ) 
    {
      dm4310_fb[1].id    = data[0] & 0x0F;
      dm4310_fb[1].error = (data[0] >> 4) & 0x0F;

      // POS: 16位，D[1]高字节，D[2]低字节
      uint16_t p_int = (uint16_t)((data[1] << 8) | data[2]);

      // VEL: 12位，D[3]全8位为高8位，D[4]高4位为低4位
      uint16_t v_int = (uint16_t)((data[3] << 4) | (data[4] >> 4));

      // TOR: 12位，D[4]低4位为高4位，D[5]全8位为低8位
      uint16_t t_int = (uint16_t)(((data[4] & 0x0F) << 8) | data[5]);

      float current_abs_rad = dm_uint_to_float(p_int, DM_P_MIN, DM_P_MAX, 16);

      if (dm4310_fb[1].is_initialized == 0) {
          dm4310_fb[1].pos_offset_rad = current_abs_rad;
          dm4310_fb[1].is_initialized = 1;
      }

      dm4310_fb[1].position_rad  = current_abs_rad - dm4310_fb[1].pos_offset_rad;
      dm4310_fb[1].position_deg  = (dm4310_fb[1].position_rad - current_abs_rad) * (180.0f / DM_PI);
      dm4310_fb[1].velocity_rads = dm_uint_to_float(v_int, DM_V_MIN, DM_V_MAX, 12);
      dm4310_fb[1].torque_Nm     = dm_uint_to_float(t_int, DM_T_MIN, DM_T_MAX, 12);
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
    // 检查电机是否已初始化
    // if (CAN_ID < 2 || CAN_ID > 3 || dm4310_fb[CAN_ID - 2].is_initialized == 0) {
    //     return;
    // }
    if (CAN_ID < 2 || CAN_ID > 3) {
        return;
    }
    
    float p_target_abs = (p_des_deg * (DM_PI / 180.0f));
    float v_limit_rad = v_des_deg * (DM_PI / 180.0f);

    // 根据参考代码，位置速度模式ID = 0x100 + CAN_ID
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
    
    // 参考代码pos_ctrl函数：直接发送浮点数的字节表示
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
    uint8_t data[4];
    
    // 根据参考代码，速度模式ID = 0x200 + CAN_ID
    header.Identifier = 0x200 + CAN_ID;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_4;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0;
    
    // 参考代码spd_ctrl函数：只发送4字节速度值
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

void DM_CAN_Disable_Motor(uint8_t CAN_ID) 
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

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

void DM_CAN_Save_Zero_Motor(uint8_t CAN_ID) 
{
    FDCAN_TxHeaderTypeDef header;
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

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


/* USER CODE END Exported functions */