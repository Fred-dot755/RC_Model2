/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Unitree */
osThreadId_t UnitreeHandle;
const osThreadAttr_t Unitree_attributes = {
  .name = "Unitree",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DM */
osThreadId_t DMHandle;
const osThreadAttr_t DM_attributes = {
  .name = "DM",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DJI */
osThreadId_t DJIHandle;
const osThreadAttr_t DJI_attributes = {
  .name = "DJI",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Remote */
osThreadId_t RemoteHandle;
const osThreadAttr_t Remote_attributes = {
  .name = "Remote",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Unitree_Function(void *argument);
void DM_Function(void *argument);
void DJI_Function(void *argument);
void Remote_Function(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Unitree */
  UnitreeHandle = osThreadNew(Unitree_Function, NULL, &Unitree_attributes);

  /* creation of DM */
  DMHandle = osThreadNew(DM_Function, NULL, &DM_attributes);

  /* creation of DJI */
  DJIHandle = osThreadNew(DJI_Function, NULL, &DJI_attributes);

  /* creation of Remote */
  RemoteHandle = osThreadNew(Remote_Function, NULL, &Remote_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    if(rc_data.btn_3 == 1 && rc_data.btn_4 == 1)
    {
      remote_mode += 1;
      osDelay(1000);
    }
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Unitree_Function */
/**
* @brief Function implementing the Unitree thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Unitree_Function */
void Unitree_Function(void *argument)
{
  /* USER CODE BEGIN Unitree_Function */
  /* Infinite loop */
  for(;;)
  {
    unitree_cmd_create(&unitree_cmd[1], 1, 1, 2.5, 0.1, -unitree_lift, 0.0, 0.0);
    unitree_communicate(1);
    osDelay(10);
    unitree_cmd_create(&unitree_cmd[3], 3, 1, 2.5, 0.1, unitree_lift, 0.0, 0.0);
    unitree_communicate(3);
    osDelay(10);
    unitree_cmd_create(&unitree_cmd[2], 2, 1, 2.5, 0.1, unitree_angle_turn, 0.0, 0.0);
    unitree_communicate(2);
    osDelay(10);
    //1负3正，2左正右负

    // unitree_cmd_create(&unitree_cmd[1], 1, 1, 0.0, 0.0, 0.0, 0.0, 0.0);
    // unitree_communicate(1);
    // osDelay(10);
    // unitree_cmd_create(&unitree_cmd[3], 3, 1, 0.0, 0.0, 0.0, 0.0, 0.0);
    // unitree_communicate(3);
    // osDelay(10);
    // unitree_cmd_create(&unitree_cmd[2], 2, 1, 0.0, 0.0, 0.0, 0.0, 0.0);
    // unitree_communicate(2);
    // osDelay(10);
    //调试用
    osDelay(5);
  }
  /* USER CODE END Unitree_Function */
}

/* USER CODE BEGIN Header_DM_Function */
/**
* @brief Function implementing the DM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DM_Function */
void DM_Function(void *argument)
{
  /* USER CODE BEGIN DM_Function */
  /* Infinite loop */
  for(;;)
  {
    DM_CAN_Send_PosVel_Mode(dm_wrist,20,2);//上正
    DM_CAN_Send_PosVel_Mode(-dm_arm,20,3);//上负

    //调试用
    // DM_CAN_Send_PosVel_Mode(0,0,2);
    // DM_CAN_Send_PosVel_Mode(0,0,3);
    osDelay(5);
  }
  /* USER CODE END DM_Function */
}

/* USER CODE BEGIN Header_DJI_Function */
/**
* @brief Function implementing the DJI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DJI_Function */
void DJI_Function(void *argument)
{
  /* USER CODE BEGIN DJI_Function */
  /* Infinite loop */
  for(;;)
  {
    FDCAN_cmd_chassis_fdcan1_0x200(pid_3508[0], pid_3508[1], pid_3508[2], pid_3508[3]);
    FDCAN_cmd_chassis_fdcan1_0x1FF(pid_3508[4], pid_3508[5], 0,0);
    osDelay(5);
  }
  /* USER CODE END DJI_Function */
}

/* USER CODE BEGIN Header_Remote_Function */
/**
* @brief Function implementing the Remote thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Remote_Function */
void Remote_Function(void *argument)
{
  /* USER CODE BEGIN Remote_Function */
  /* Infinite loop */
  for(;;)
  {
    // if(rc_data.distance != 0)
    // {
    //     if(rc_data.angle > -45 && rc_data.angle < 45)
    //     {
    //       unitree_lift += 1;
    //       osDelay(100);
    //     }
    //     else if(rc_data.angle > 45 && rc_data.angle < 135)
    //     {
    //       unitree_angle_turn += 2;
    //       osDelay(100);
    //     }
    //     else if((rc_data.angle > 135 && rc_data.angle < 180) || (rc_data.angle > -180 && rc_data.angle < -135))
    //     {
    //       unitree_lift -= 1;
    //       osDelay(100);
    //     }
    //     else if(rc_data.angle > -135 && rc_data.angle < -45)
    //     {
    //       unitree_angle_turn -= 2;
    //       osDelay(100);
    //     }
        
    // }
    // if(rc_data.btn_1 == 1)
    // {
    //   dm_arm += 2;
    //   osDelay(100);
    // }
    // if(rc_data.btn_2 == 1)
    // {
    //   dm_arm -= 2;
    //   osDelay(100);
    // }
    // if(rc_data.btn_3 == 1)
    // {
    //   dm_wrist += 2;
    //   osDelay(100);
    // }
    // if(rc_data.btn_4 == 1)
    // {
    //   dm_wrist -= 2;
    //   osDelay(100);
    // }
    // 机械臂


    if(remote_mode %2 == 0)
    {
      if(rc_data.distance != 0 || rc_data.btn_1 != 1 || rc_data.btn_2 != 1 || rc_data.btn_3 != 1 || rc_data.btn_4 != 1)
      {
          angle = rc_data.angle;
          speed = rc_data.distance * 2000;

          lift = rc_data.btn_1 * 10000;
          down = -rc_data.btn_2 * 10000;

          span = rc_data.btn_3 * 2000;
          span_t = -rc_data.btn_4 * 2000;

          chassic_control(angle,speed,span+span_t);
      }
    }
    else if (remote_mode %2 == 1)
    {
        if(rc_data.distance != 0)
        {
            if(rc_data.angle > -45 && rc_data.angle < 45)
            {
              unitree_lift += 1;
              osDelay(100);
            }
            else if(rc_data.angle > 45 && rc_data.angle < 135)
            {
              unitree_angle_turn += 2;
              osDelay(100);
            }
            else if((rc_data.angle > 135 && rc_data.angle < 180) || (rc_data.angle > -180 && rc_data.angle < -135))
            {
              unitree_lift -= 1;
              osDelay(100);
            }
            else if(rc_data.angle > -135 && rc_data.angle < -45)
            {
              unitree_angle_turn -= 2;
              osDelay(100);
            }
        
        }
        if(rc_data.btn_1 == 1)
        {
          dm_arm += 2;
          osDelay(100);
        }
        if(rc_data.btn_2 == 1)
        {
          dm_arm -= 2;
          osDelay(100);
        }
        if(rc_data.btn_3 == 1)
        {
          dm_wrist += 2;
          osDelay(100);
        }
        if(rc_data.btn_4 == 1)
        {
          dm_wrist -= 2;
          osDelay(100);
        }
    }


    osDelay(5);
  }
  /* USER CODE END Remote_Function */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

