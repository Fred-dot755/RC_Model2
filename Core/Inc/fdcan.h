/* USER CODE BEGIN Header */
//written by Fred Xiao

/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

extern FDCAN_HandleTypeDef hfdcan2;

extern FDCAN_HandleTypeDef hfdcan3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);
void MX_FDCAN2_Init(void);
void MX_FDCAN3_Init(void);

/* USER CODE BEGIN Prototypes */

// FDCAN Filter and Communication Functions
void fdcan_filter_init(void);
void fdcan1_filter_init(void);
void fdcan2_filter_init(void);
void fdcan3_filter_init(void);

// Chassis motor control functions for FDCAN1
void FDCAN_cmd_chassis_fdcan1_0x200(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void FDCAN_cmd_chassis_fdcan1_0x1FF(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

// Chassis motor control functions for FDCAN2
void FDCAN_cmd_chassis_fdcan2_0x200(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void FDCAN_cmd_chassis_fdcan2_0x1FF(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

// RX Callback function (to be implemented in user code)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

// External variables declaration for send data buffers
extern uint8_t chassis_fdcan1_send_data_0x200[8];
extern uint8_t chassis_fdcan1_send_data_0x1FF[8];
extern uint8_t chassis_fdcan2_send_data_0x200[8];
extern uint8_t chassis_fdcan2_send_data_0x1FF[8];

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

