/* USER CODE BEGIN Header */
//written by Fred Xiao

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

static void dm_get_feedback_ranges(uint8_t motor_id,
                                   float *v_min,
                                   float *v_max,
                                   float *t_min,
                                   float *t_max)
{
    if ((motor_id == 2U) || (motor_id == 6U) || (motor_id == 7U)) {
        *v_min = DM4310_V_MIN;
        *v_max = DM4310_V_MAX;
        *t_min = DM4310_T_MIN;
        *t_max = DM4310_T_MAX;
    } else {
        *v_min = DM8009_V_MIN;
        *v_max = DM8009_V_MAX;
        *t_min = DM8009_T_MIN;
        *t_max = DM8009_T_MAX;
    }
}

/* USER CODE END Private defines */

/* USER CODE BEGIN Private variables */
DM4310_Feedback_t dm4310_fb[DM_MOTOR_COUNT] = {0};

/* USER CODE END Private variables */

/* USER CODE BEGIN Exported functions */

/**
  * @brief  达妙电机初始化
  * @retval None
  */
void DM_Motor_Init(void)
{
    for (uint32_t i = 0U; i < DM_MOTOR_COUNT; i++) {
        dm4310_fb[i] = (DM4310_Feedback_t){0};
    }

    for (uint8_t motor_id = DM_FIRST_MOTOR_ID;
         motor_id < (DM_FIRST_MOTOR_ID + DM_MOTOR_COUNT);
         motor_id++) {
        if (!DM_MOTOR_IS_ACTIVE(motor_id)) {
            continue;
        }
        DM_CAN_Enable_Motor(motor_id);
        HAL_Delay(100);
    }

    for (uint8_t motor_id = DM_FIRST_MOTOR_ID;
         motor_id < (DM_FIRST_MOTOR_ID + DM_MOTOR_COUNT);
         motor_id++) {
        if (!DM_MOTOR_IS_ACTIVE(motor_id)) {
            continue;
        }
        DM_CAN_Save_Zero_Motor(motor_id);
        HAL_Delay(100);
    }
}

/**
  * @brief  处理接收到的CAN消息
  * @param  StdId: 标准ID
  * @param  data: 数据数组
  * @retval None
  */
void DM_Process_Rx_Message(uint32_t StdId, uint8_t* data)
{
    if (data == NULL) {
        return;
    }

    uint8_t motor_id = data[0] & 0x0FU;
    if ((motor_id < DM_FIRST_MOTOR_ID) ||
        (motor_id >= (DM_FIRST_MOTOR_ID + DM_MOTOR_COUNT))) {
        return;
    }

    /*
     * DM feedback uses byte 0 to identify the motor. The CAN identifier is
     * either the configured master ID or, in some setups, the motor ID.
     */
    if ((StdId != DM_FEEDBACK_ID) && (StdId != motor_id)) {
        return;
    }

    uint32_t motor_index = motor_id - DM_FIRST_MOTOR_ID;
    DM4310_Feedback_t *feedback = &dm4310_fb[motor_index];

    feedback->id = motor_id;
    feedback->error = (data[0] >> 4) & 0x0FU;

    feedback->pos_raw = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
    feedback->vel_raw = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
    feedback->tor_raw = (uint16_t)(((uint16_t)(data[4] & 0x0FU) << 8) | data[5]);

    float current_abs_rad =
        dm_uint_to_float(feedback->pos_raw, DM_P_MIN, DM_P_MAX, 16);

    if (feedback->is_initialized == 0U) {
        feedback->pos_offset_rad = current_abs_rad;
        feedback->is_initialized = 1U;
    }

    feedback->position_rad = current_abs_rad - feedback->pos_offset_rad;
    feedback->position_deg = -feedback->position_rad * (180.0f / DM_PI);

    float v_min;
    float v_max;
    float t_min;
    float t_max;
    dm_get_feedback_ranges(motor_id, &v_min, &v_max, &t_min, &t_max);

    feedback->velocity_rads =
        dm_uint_to_float(feedback->vel_raw, v_min, v_max, 12);
    feedback->torque_Nm =
        dm_uint_to_float(feedback->tor_raw, t_min, t_max, 12);
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
    // if (CAN_ID < 2 || CAN_ID > 3) {
    //     return;
    // }
    
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
