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
static uint8_t kfs_count_total_ready = 0U;

#define ANGLE_RING_MAX_SPEED_DPS       120.0f
#define ANGLE_RING_MIN_DURATION_S      0.25f
#define ANGLE_RING_TARGET_EPSILON_DEG  0.1f

static float angle_ring_absf(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static float angle_ring_normalize_deg(float angle)
{
  while(angle > 180.0f)
  {
    angle -= 360.0f;
  }
  while(angle < -180.0f)
  {
    angle += 360.0f;
  }
  return angle;
}


static int calc_kfs_count_total(void)
{
  int total = 0;
  uint8_t count = visual_data.meilin_count;

  if(count > 10U)
  {
    count = 10U;
  }

  for(uint8_t i = 0U; i < count; i++)
  {
    MeilinPointCmd point = visual_data.meilin_points[i];

    if(point.has_true_kfs == 1U || point.cell == 13U || point.cell == 15U)
    {
      total++;
    }
  }

  return total;
}
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
/* Definitions for DH_C */
osThreadId_t DH_CHandle;
const osThreadAttr_t DH_C_attributes = {
  .name = "DH_C",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Remote_mode */
osThreadId_t Remote_modeHandle;
const osThreadAttr_t Remote_mode_attributes = {
  .name = "Remote_mode",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for inverseKinemati */
osThreadId_t inverseKinematiHandle;
const osThreadAttr_t inverseKinemati_attributes = {
  .name = "inverseKinemati",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Angle_ring */
osThreadId_t Angle_ringHandle;
const osThreadAttr_t Angle_ring_attributes = {
  .name = "Angle_ring",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for L1_Mode */
osThreadId_t L1_ModeHandle;
const osThreadAttr_t L1_Mode_attributes = {
  .name = "L1_Mode",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Lift_Mode */
osThreadId_t Lift_ModeHandle;
const osThreadAttr_t Lift_Mode_attributes = {
  .name = "Lift_Mode",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for One_Area */
osThreadId_t One_AreaHandle;
const osThreadAttr_t One_Area_attributes = {
  .name = "One_Area",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Two_Area */
osThreadId_t Two_AreaHandle;
const osThreadAttr_t Two_Area_attributes = {
  .name = "Two_Area",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Three_Area */
osThreadId_t Three_AreaHandle;
const osThreadAttr_t Three_Area_attributes = {
  .name = "Three_Area",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Mid360 */
osThreadId_t Mid360Handle;
const osThreadAttr_t Mid360_attributes = {
  .name = "Mid360",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Track_Mode */
osThreadId_t Track_ModeHandle;
const osThreadAttr_t Track_Mode_attributes = {
  .name = "Track_Mode",
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
void DH_C_Function(void *argument);
void Remote_mode_function(void *argument);
void inverseKinematics_Function(void *argument);
void Angle_ring_Function(void *argument);
void L1_Mode_Function(void *argument);
void Lift_Mode_Function(void *argument);
void One_Area_Function(void *argument);
void Two_Area_Function(void *argument);
void Three_Area_Function(void *argument);
void Mid360_Function(void *argument);
void Track_Mode_Function(void *argument);

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

  /* creation of DH_C */
  DH_CHandle = osThreadNew(DH_C_Function, NULL, &DH_C_attributes);

  /* creation of Remote_mode */
  Remote_modeHandle = osThreadNew(Remote_mode_function, NULL, &Remote_mode_attributes);

  /* creation of inverseKinemati */
  inverseKinematiHandle = osThreadNew(inverseKinematics_Function, NULL, &inverseKinemati_attributes);

  /* creation of Angle_ring */
  Angle_ringHandle = osThreadNew(Angle_ring_Function, NULL, &Angle_ring_attributes);

  /* creation of L1_Mode */
  L1_ModeHandle = osThreadNew(L1_Mode_Function, NULL, &L1_Mode_attributes);

  /* creation of Lift_Mode */
  Lift_ModeHandle = osThreadNew(Lift_Mode_Function, NULL, &Lift_Mode_attributes);

  /* creation of One_Area */
  One_AreaHandle = osThreadNew(One_Area_Function, NULL, &One_Area_attributes);

  /* creation of Two_Area */
  Two_AreaHandle = osThreadNew(Two_Area_Function, NULL, &Two_Area_attributes);

  /* creation of Three_Area */
  Three_AreaHandle = osThreadNew(Three_Area_Function, NULL, &Three_Area_attributes);

  /* creation of Mid360 */
  Mid360Handle = osThreadNew(Mid360_Function, NULL, &Mid360_attributes);

  /* creation of Track_Mode */
  Track_ModeHandle = osThreadNew(Track_Mode_Function, NULL, &Track_Mode_attributes);

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
    if(visual_data.hmi_start != 1)
    {
      kfs_count_total_ready = 0U;
      R2_Extern.kfs_count_total = 0;
    }
    else if(kfs_count_total_ready == 0U)
    {
      R2_Extern.kfs_count_total = calc_kfs_count_total();
      kfs_count_total_ready = 1U;
    }

    set_yuntai_angle(R2_Extern.pitch_angle, R2_Extern.yaw_angle);
    meic_protocol_send_packet_dma(&huart10, R2_Extern.pitch_angle, R2_Extern.yaw_angle ,R2_Extern.work_mode);
    // set_yuntai_angle(10, -10);
    // meic_protocol_send_packet_dma(&huart10, 10,-10,2);
    // set_yuntai_angle(45, -10);
    // meic_protocol_send_packet_dma(&huart10, 45,-10,2);

    // if(guangdian == 1)
    // {
    //   R2_Extern.lingshi_flag = 10;
    // }
    // else
    // {
    //   R2_Extern.lingshi_flag = 0;
    // }

    osDelay(5);
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
    //小臂变为宇树电机了
    // unitree_cmd_create(&unitree_cmd[3], 3, 1, 10.7, 0.1, now_position.output_angle3, 0.0, 0.0);
    // unitree_communicate(3);
    // osDelay(10);

    unitree_cmd_create(&unitree_cmd[3], 3, 1, 0.0, 0.0, 0.0, 0.0, 0.0);
    unitree_communicate(3);
    osDelay(10);


    // if(R2_Extern.angle3 == 0)
    // {
    //   unitree_cmd_create(&unitree_cmd[3], 3, 1, 0.0, 0.0, 0.0, 0.0, 0.0);
    //   unitree_communicate(3);
    //   osDelay(10);
    // }
    // else
    // {
    //   unitree_cmd_create(&unitree_cmd[3], 3, 1, 10.7, 0.1, now_position.output_angle3, 0.0, 0.0);
    //   unitree_communicate(3);
    //   osDelay(10);
    // }

    //调试用
    osDelay(3);
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
    DM_CAN_Enable_Motor(2);
    DM_CAN_Enable_Motor(3);
    DM_CAN_Enable_Motor(4);
    DM_CAN_Enable_Motor(5);
    DM_CAN_Enable_Motor(6);
    DM_CAN_Enable_Motor(7);

    DM_CAN_Send_PosVel_Mode(-(-R2_Extern.angle4  - unitree_pos[1] + dm4310_fb[1].position_deg) * 2.5,200,2);//上正
    DM_CAN_Send_PosVel_Mode(-R2_Extern.angle2,40,3);//上负    大臂电机改为8009了
      DM_CAN_Send_PosVel_Mode(R2_Extern.lift,1200,4);
      DM_CAN_Send_PosVel_Mode(R2_Extern.lift,1200,5);
    DM_CAN_Send_PosVel_Mode(R2_Extern.angle5,1200,6);//爪子
    DM_CAN_Send_PosVel_Mode(R2_Extern.angle1 * 1.625f,100,7);//云台：angle1为[-180, 180)相对角度

    //调试用
    // DM_CAN_Send_PosVel_Mode(0,0,2);
    // DM_CAN_Send_PosVel_Mode(0,0,3);
    // DM_CAN_Send_PosVel_Mode(0,0,4);
    // DM_CAN_Send_PosVel_Mode(0,0,5);
    // DM_CAN_Send_PosVel_Mode(0,0,6);
    // DM_CAN_Send_PosVel_Mode(0,0,7);
    osDelay(3);
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
    FDCAN_cmd_chassis_fdcan1_0x1FF(pid_3508[4], pid_3508[5], pid_3508[6], pid_3508[7]);
    FDCAN_cmd_chassis_fdcan3_0x200(pid_3508[8], pid_3508[9], pid_3508[10], pid_3508[11]);

    osDelay(3);
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
    // R2_Extern.lift_mood = 1;
    // R2_Extern.angle5 = 92;
    // R2_Extern.angle = rc_data.angle;
    // R2_Extern.speed = rc_data.distance *2;
    // R2_Extern.lingshi_flag = guangdian;

    // if(rc_data.btn_1 == 1)
    // {
    //   // fangkuang_open_1();
    //   qibeng_open();
    //   // chsaaic_front_up();
    //   // R2_Extern.angle2 += 1;
    //   // zhuazi_open();
    //   // R2_Extern.angle2 += 1;
    //   // fangkuang_open();
    //   // R2_Extern.lift_mood = 1;
    //   osDelay(200);
    // }
    // if(rc_data.btn_2 == 1)
    // {
    //   // fangkuang_close_1();
    //   qibeng_close();
    //   // chsaaic_front_down();
    //   // R2_Extern.angle2 -= 1;
    //   // zhuazi_close();
    //   // R2_Extern.angle2 -= 1;
    //   // fangkuang_close();
    //   // R2_Extern.lift_mood = 1;
    //   osDelay(200);
    //   // chsaaic_front_down();
    //   // R2_Extern.angle2 -= 1;
    //   // zhuazi_close();
    //   // R2_Extern.angle2 -= 1;
    //   // fangkuang_close();
    //   // put_kfs_3();
    //   osDelay(200);
    // }
    // if(rc_data.btn_3 == 1)
    // {
    //   fangkuang_open_2();
    //   // R2_Extern.angle3 += 1;
    //   // chsaaic_behind_up();
    //   // R2_Extern.angle3 += 1;
    //   // put_kfs_1();
    //   osDelay(200);
    // }
    // if(rc_data.btn_4 == 1)
    // {
    //   fangkuang_close_2();
    //   // R2_Extern.angle3 -= 1;
    //   // chsaaic_behind_down();
    //   // R2_Extern.angle3 -= 1;
    //   // put_kfs_2();
    //   osDelay(200);
    // }

    // if(R2_Extern.get_init == 0)
    // {
    //   // R2_Extern.angle2 = angle_2;
    //   // R2_Extern.angle3 = angle_3;
    //   R2_Extern.angle2 = 30;
    //   R2_Extern.angle3 = 45;
    //   R2_Extern.angle4 = 0;
    //   // R2_Extern.lift_mood = 1;
    //   // osDelay(1000);
    //   // chsaaic_behind_up();
    //   // chsaaic_front_up();
    //   // R2_Extern.angle2 = 145;
    //   // R2_Extern.angle3 = 80;
    //   // R2_Extern.angle4 = 0;
    //   if(unitree_pos[1]>= R2_Extern.angle3 - 5 && unitree_pos[1] <= R2_Extern.angle3 + 5 && dm4310_fb[1].position_deg >= R2_Extern.angle2 - 5 && dm4310_fb[1].position_deg <= R2_Extern.angle2 + 5)
    //   {
    //     R2_Extern.get_init = 1;
    //   }
    // }


    // if(rc_data.btn_1 == 1)
    // {
    //   R2_Extern.angle_balance_target += 1;
    //   // chsaaic_behind_up();
    //   // fangkuang_open();
    //   // osDelay(50);
    // }
    // if(rc_data.btn_2 == 1)
    // {
    //   R2_Extern.angle_balance_target -= 1;
    //   // chsaaic_behind_down();
    //   // fangkuang_close();
    //   // osDelay(50);
    // }
    // if(rc_data.btn_3 == 1)
    // {
    //   // fangkuang_open();
    //   R2_Extern.Area2_flag = 1;
    //   // R2_Extern.angle5 = 95;
    //   // chsaaic_front_up();
    // }
    // if(rc_data.btn_4 == 1)
    // {
    //   // fangkuang_close();
    //   R2_Extern.Area2_flag = 5;
    //   // R2_Extern.angle5 = 130;
    //   // chsaaic_front_down();
    // }

    //     if(rc_data.btn_1 == 1 && rc_data.btn_2 == 1)
    // {
    //   // R2_Extern.angle5 = -80;
    //   // osDelay(100);
    //   chsaaic_behind_down();
    //   osDelay(200);
    // }
    // else if(rc_data.btn_1 == 1 && rc_data.btn_3 == 1)
    // {
    //   // R2_Extern.angle5 = 0;
    //   // osDelay(100);
    //   chsaaic_behind_up();
    //   osDelay(200);
    // }
    // else if(rc_data.btn_2 == 1 && rc_data.btn_3 == 1)
    // {
    //   // grap_on();
    //   // osDelay(100);
    //   chsaaic_front_down();
    //   osDelay(200);
    // }
    // else if(rc_data.btn_2 == 1 && rc_data.btn_4 == 1)
    // {
    //   // grap_off();
    //   // osDelay(100);
    //   chsaaic_front_up();
    //   osDelay(200);
    // }
    // else if(rc_data.btn_3 == 1 && rc_data.btn_4 == 1)
    // {
    //   R2_Extern.lift_mood = 1;
    //   osDelay(200);
    // }

    
//       switch (R2_Extern.Area2_flag)
//       {
//       case 0:
//           // R2_Extern.angle = 0;
//           // R2_Extern.speed = 0;
//           // R2_Extern.angle_balance_target = 0;
//         break;

//       case 1:
//           if(R2_Extern.lift_mood == 0)
//           {
//             R2_Extern.lift_mood = 1;
//             osDelay(2000);
//           }
//           R2_Extern.angle = 0;
//           R2_Extern.speed = 0.3;
//           if(Area_Flag.qian_dis == 1 && R2_Extern.lift_mood == 1)
//           {
//               R2_Extern.angle = 0;
//               R2_Extern.speed = 0;
//               R2_Extern.Area2_flag = 2;
//           }
//         break;

//       case 2:
//           if(R2_Extern.chsaaic_behind_flag == 0)
//           {
//             chsaaic_behind_up();
//             R2_Extern.chsaaic_behind_flag = 1;
//             osDelay(500);
//           }
//           R2_Extern.lift_mood = 2;
//           osDelay(2000);
//           R2_Extern.Area2_flag = 3;
//         break;

//       case 3:
//           now_mood.mood = 1;
//           R2_Extern.angle = 0;
//           R2_Extern.speed = 0.3;
//           if(Area_Flag.hou_dis == 1)
//           {
//             now_mood.mood = 0;
//             R2_Extern.angle = 0;
//             R2_Extern.speed = 0;
//             chsaaic_behind_down();
//             R2_Extern.lift_mood = 0;
//             R2_Extern.chsaaic_behind_flag = 0;
//             osDelay(1000);
//             R2_Extern.Area2_flag = 4;
//           }
//         break;

//       case 4:
//           R2_Extern.angle = 0;
//           R2_Extern.speed = 0.3;
//           osDelay(500);
//           R2_Extern.Area2_flag = 0;
//           R2_Extern.complete_taijie_flag = 1;

//         break;
// //上台阶1到4

//       case 5:
//           if(R2_Extern.lift_mood == 0)
//           {
//             R2_Extern.lift_mood = 1;
//             osDelay(1000);
//           }
//           if(R2_Extern.chack_yaw_flag == 1)
//           {
//             R2_Extern.angle = 0;
//             R2_Extern.speed = 0.2;
//           }
//           else
//           {
//             R2_Extern.angle = 0;
//             R2_Extern.speed = 0;
//           }

//           if(Area_Flag.qian_dis == 2 && R2_Extern.lift_mood == 1)
//           {
//               R2_Extern.angle = 0;
//               R2_Extern.speed = 0;
//               R2_Extern.Area2_flag = 6;
//           }
//         break;

//       case 6:
//           if(R2_Extern.lift_mood == 1)
//           {
//             R2_Extern.lift_mood = 2;
//             chsaaic_front_up();
//             osDelay(2500);
//           }
//           R2_Extern.angle = 0;
//           R2_Extern.speed = 0.2;
//           now_mood.mood = 1;

//           if(Area_Flag.hou_dis == 0 && R2_Extern.lift_mood == 2)
//           {
//             now_mood.mood = 0;
//             R2_Extern.angle = 0;
//             R2_Extern.speed = 0;
//             osDelay(100);
//             R2_Extern.Area2_flag = 7;
//           }
//       break;

//       case 7:
//             R2_Extern.lift_mood = 1;
//             osDelay(2000);
//             chsaaic_front_down();
//             R2_Extern.Area2_flag = 8;

//       break;

//       case 8:
//           R2_Extern.angle = 0;
//           R2_Extern.speed = 0.3;
//           osDelay(1000);
//           R2_Extern.lift_mood = 0;
//           R2_Extern.Area2_flag = 0;
//           R2_Extern.complete_taijie_flag = 1;
//           if(R2_Extern.complete_erqu_flag == 1)
//           {
//             R2_Extern.start_sanqugoon_flag = 1;
//             R2_Extern.bool_meilin_flag = 1;
//           }
//       break;
// //下台阶5到8

//       default:
//         break;
//     }




    // if(R2_Extern.lingshi_flag == 0)
    // {
    //   osDelay(2000);
    //   zhuazi_mode_1_5();
    //   osDelay(1000);
    //   zhuazi_mode_2();
    //   osDelay(1000);
    //   zhuazi_mode_3();
    //   osDelay(1000);
    //   zhuazi_mode_4();
    //   osDelay(1000);
    //   zhuazi_mode_1_5();
    //   osDelay(2000);
    //   zhuazi_mode_6();
    //   osDelay(1000);
    //   zhuazi_mode_7();
    //   osDelay(1000);
    //   R2_Extern.lingshi_flag = 1;
    // }

    // chassic_control(R2_Extern.angle,R2_Extern.speed,R2_Extern.span+(-R2_Extern.error_balance*200));


    osDelay(5);
  }
  /* USER CODE END Remote_Function */
}

/* USER CODE BEGIN Header_DH_C_Function */
/**
* @brief Function implementing the DH_C thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DH_C_Function */
void DH_C_Function(void *argument)
{
  /* USER CODE BEGIN DH_C_Function */
  /* Infinite loop */
  for(;;)
  {
    computeDHTransform(dm4310_fb[5].position_deg, dm4310_fb[1].position_deg , unitree_pos[1], &R2_Extern.x_t, &R2_Extern.y_t, &R2_Extern.z_t);
    // inverseKinematics(x,y,z,&angle1,&angle2,&angle3);
    osDelay(5);
  }
  /* USER CODE END DH_C_Function */
}

/* USER CODE BEGIN Header_Remote_mode_function */
/**
* @brief Function implementing the Remote_mode thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Remote_mode_function */
void Remote_mode_function(void *argument)
{
  /* USER CODE BEGIN Remote_mode_function */
  /* Infinite loop */
  for(;;)
  {
    if(L1_Sensor1.distance_mm <= 190)
    {
      Area_Flag.qian_dis = 1;//低
    }
    else if(L1_Sensor1.distance_mm >= 450)
    {
      Area_Flag.qian_dis = 2;//高
    }
    else
    {
      Area_Flag.qian_dis = 0;
    }

    if(L1_Sensor2.distance_mm <= 280)
    {
      Area_Flag.hou_dis = 1;//低
    }
    else if(L1_Sensor2.distance_mm >= 550)
    {
      Area_Flag.hou_dis = 2;//高
    }
    else
    {
      Area_Flag.hou_dis = 0;
    }

    //检测姿态
    if(fabsf(R2_Extern.error_balance) < 2.0f)
    {
      R2_Extern.chack_yaw_flag = 1;
    }
    else
    {
      R2_Extern.chack_yaw_flag = 0;
    }

    //检测定位
    uint8_t cell = visual_data.meilin_points[R2_Extern.meilin_count_flag].cell;

    if(cell >= 1 && cell <= 12)
    {
      check_dingwei(visual_data.x_map, visual_data.y_map, cell - 1);
    }

    //是否看到矿
    if(visual_data.bool_getKFS == 1)
    {
      R2_Extern.KFS_Get_flag = 1;
    }
    else
    {
      R2_Extern.KFS_Get_flag = 0;
    }

    //云台
    if(abs(R2_Extern.angle1 - dm4310_fb[5].position_deg) < 5)
    {
      R2_Extern.check_angle1_flag = 1;
    }
    else
    {
      R2_Extern.check_angle1_flag = 0;
    }

    //大臂
    if(abs(R2_Extern.angle2 - dm4310_fb[1].position_deg) < 5)
    {
      R2_Extern.check_angle2_flag = 1;
    }
    else
    {
      R2_Extern.check_angle2_flag = 0;
    }

    //小臂
    if(abs(R2_Extern.angle3 - unitree_pos[1]) < 5)
    {
      R2_Extern.check_angle3_flag = 1;
    }
    else
    {
      R2_Extern.check_angle3_flag = 0;
    }

    //手腕
    if(abs(R2_Extern.angle4 - dm4310_fb[0].position_deg) < 5)
    {
      R2_Extern.check_angle4_flag = 1;
    }
    else
    {
      R2_Extern.check_angle4_flag = 0;
    }

    //爪子
    if(abs(R2_Extern.angle5 + dm4310_fb[4].position_deg) < 5)
    {
      R2_Extern.check_angle5_flag = 1;
    }
    else
    {
      R2_Extern.check_angle5_flag = 0;
    }

    osDelay(1);
  }
  /* USER CODE END Remote_mode_function */
}

/* USER CODE BEGIN Header_inverseKinematics_Function */
/**
* @brief Function implementing the inverseKinemati thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_inverseKinematics_Function */
void inverseKinematics_Function(void *argument)
{
  /* USER CODE BEGIN inverseKinematics_Function */
  /* Infinite loop */
  for(;;)
  {
    arm_unitree_planning_update(R2_Extern.angle1, R2_Extern.angle3);
    osDelay(25);
  }
  /* USER CODE END inverseKinematics_Function */
}

/* USER CODE BEGIN Header_Angle_ring_Function */
/**
* @brief Function implementing the Angle_ring thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Angle_ring_Function */
void Angle_ring_Function(void *argument)
{
  /* USER CODE BEGIN Angle_ring_Function */
  static float prev_hipnuc_angle_z = 0.0f;
  static float total_wrap_angle_z = 0.0f;
  static float plan_start_angle = 0.0f;
  static float plan_delta_angle = 0.0f;
  static float plan_target_angle = 0.0f;
  static uint32_t plan_start_tick = 0U;
  static float plan_duration_s = 0.0f;
  static uint8_t plan_active = 0U;
  /* Infinite loop */
  for(;;)
  {
    float current_angle = ops.HIPNUCAngleZ;
    float delta = current_angle - prev_hipnuc_angle_z;
    if (delta > 180.0f) {
        total_wrap_angle_z -= 360.0f;
    } else if (delta < -180.0f) {
        total_wrap_angle_z += 360.0f;
    }
    prev_hipnuc_angle_z = current_angle;
    float continuous_angle = current_angle + total_wrap_angle_z;

    float target_diff = angle_ring_normalize_deg(R2_Extern.angle_balance_target - plan_target_angle);
    if(plan_active == 0U || angle_ring_absf(target_diff) > ANGLE_RING_TARGET_EPSILON_DEG)
    {
      plan_start_angle = R2_Extern.angle_balance;
      plan_delta_angle = angle_ring_normalize_deg(R2_Extern.angle_balance_target - plan_start_angle);
      plan_target_angle = plan_start_angle + plan_delta_angle;
      plan_start_tick = HAL_GetTick();

      float distance = angle_ring_absf(plan_delta_angle);
      if(distance <= ANGLE_RING_TARGET_EPSILON_DEG)
      {
        R2_Extern.angle_balance = R2_Extern.angle_balance_target;
        plan_duration_s = 0.0f;
        plan_active = 0U;
      }
      else
      {
        plan_duration_s = 1.875f * distance / ANGLE_RING_MAX_SPEED_DPS;
        if(plan_duration_s < ANGLE_RING_MIN_DURATION_S)
        {
          plan_duration_s = ANGLE_RING_MIN_DURATION_S;
        }
        plan_active = 1U;
      }
    }

    if(plan_active != 0U)
    {
      float elapsed_s = (float)(HAL_GetTick() - plan_start_tick) * 0.001f;
      if(elapsed_s >= plan_duration_s)
      {
        R2_Extern.angle_balance = plan_target_angle;
        plan_active = 0U;
      }
      else
      {
        float u = elapsed_s / plan_duration_s;
        float u2 = u * u;
        float u3 = u2 * u;
        float smooth = (10.0f * u3) - (15.0f * u2 * u2) + (6.0f * u3 * u2);
        R2_Extern.angle_balance = plan_start_angle + plan_delta_angle * smooth;
      }
    }

    R2_Extern.angle_balance = angle_ring_normalize_deg(R2_Extern.angle_balance);

    // 将角度误差归一化到 [-180, 180]，确保以劣弧(最短路径)方向旋转
    R2_Extern.error_balance = fmodf(R2_Extern.angle_balance - continuous_angle, 360.0f);
    if (R2_Extern.error_balance > 180.0f) R2_Extern.error_balance -= 360.0f;
    if (R2_Extern.error_balance < -180.0f) R2_Extern.error_balance += 360.0f;
    osDelay(5);
  }
  /* USER CODE END Angle_ring_Function */
}

/* USER CODE BEGIN Header_L1_Mode_Function */
/**
* @brief Function implementing the L1_Mode thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_L1_Mode_Function */
void L1_Mode_Function(void *argument)
{
  /* USER CODE BEGIN L1_Mode_Function */
  /* Infinite loop */
  for(;;)
  {
    L1_Control(&L1_Sensor1, L1_MODE_FAST);
    L1_Control(&L1_Sensor2, L1_MODE_FAST);
    osDelay(5);
  }
  /* USER CODE END L1_Mode_Function */
}

/* USER CODE BEGIN Header_Lift_Mode_Function */
/**
* @brief Function implementing the Lift_Mode thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Lift_Mode_Function */
void Lift_Mode_Function(void *argument)
{
  /* USER CODE BEGIN Lift_Mode_Function */
  /* Infinite loop */
  for(;;)
  {
    if(R2_Extern.lift_mood == 1)
    {
      // while(R2_Extern.lift <= 400)
      // {
      //   R2_Extern.lift += 1;
      //   osDelay(1);
      // }
      // R2_Extern.lift = 470;
      R2_Extern.lift = 485;
    }
    else if(R2_Extern.lift_mood == 0)
    {
      // while(R2_Extern.lift >= 20)
      // {
      //   R2_Extern.lift -= 1;
      //   osDelay(1);
      // }
      // if(R2_Extern.lift <= 20)
      // {
      //   R2_Extern.lift = 20;
      // }
      R2_Extern.lift = 100;
    }
    else if (R2_Extern.lift_mood == 2)
    {
      R2_Extern.lift = 0;
    }
    osDelay(5);
  }
  /* USER CODE END Lift_Mode_Function */
}

/* USER CODE BEGIN Header_One_Area_Function */
/**
* @brief Function implementing the One_Area thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_One_Area_Function */
void One_Area_Function(void *argument)
{
  /* USER CODE BEGIN One_Area_Function */
  /* Infinite loop */
  for(;;)
  {
    if(visual_data.hmi_start != 1)
    {
      R2_Extern.angle = 0;
      R2_Extern.speed = 0;
      osDelay(5);
      continue;
    }

    if(visual_data.workl_mode == 1 && R2_Extern.Area1_1_flag == 1)
    {
      const float (*area_1_dt35)[2] = (R2_Extern.Track_flag == 0) ? area_1_dt35_red : area_1_dt35_blue;

      switch(R2_Extern.Area1_flag)
      {
        case 0:
        // R2_Extern.Area1_id = visual_data.targetid;
        // R2_Extern.Area1_id = 1;
        if(R2_Extern.Area1_qukuang_flag == 0)
        {
          if(R2_Extern.Track_flag == 0)
          {
            R2_Extern.Area1_id = 2;
          }
          else
          {
            R2_Extern.Area1_id = 6;
          }
          R2_Extern.Area1_qukuang_flag = 1;
        }

        
        if(R2_Extern.Area1_id == 0)
        {
          break;
        }

        if(R2_Extern.Track_flag == 0)
        {
          R2_Extern.angle_balance_target = -90;
        }
        else if(R2_Extern.Track_flag == 1)
        {
          R2_Extern.angle_balance_target = 90;
        }
        if(R2_Extern.chack_yaw_flag == 1)
        {

          R2_Extern.Area1_flag = 1;
        }
        break;

        case 1: 
          R2_Extern.angle = 0;
          R2_Extern.speed = 0;
          R2_Extern.angle5 = 140;

          if(R2_Extern.check_angle5_flag == 1)
          {
            R2_Extern.Area1_flag = 2;
          }
        break;

        case 2:
          zhuazi_open();
          quzhua(area_1_dt35[R2_Extern.Area1_id-1][0], area_1_dt35[R2_Extern.Area1_id-1][1]);
          if(R2_Extern.Area1_2_flag == 1)
          {
            R2_Extern.Area1_flag = 3;
          }
        break;

        case 3:
        R2_Extern.angle5 = 90;
        osDelay(500);

          R2_Extern.Area1_flag = 4;
        break;

        case 4:
            zhuazi_close();
            osDelay(200);
            R2_Extern.angle5 = 140;
            R2_Extern.Area1_flag = 5;
        break;

        case 5:
          if(R2_Extern.check_angle5_flag == 1)
          {

            if(guangdian == 0)
            {
              osDelay(200);
              if(guangdian == 0)
              {
                R2_Extern.Area1_flag = 6;
              }
            }
            else
            {
              if(R2_Extern.Track_flag == 0)
              {
                R2_Extern.Area1_id += 1;
                R2_Extern.Area1_flag = 2;
              }
              else
              {
                R2_Extern.Area1_id -= 1;
                R2_Extern.Area1_flag = 2;
              }
            }
            
          }
        break;

        case 6:
          
          if(R2_Extern.Track_flag == 0)
          {
            quzhua(area_1_dt35[1][0]+100, area_1_dt35[2][1]+400);
          }
          else
          {
            quzhua(area_1_dt35[4][0]+100, area_1_dt35[5][1]+400);
          }

          if(R2_Extern.Area1_2_flag == 1)
          {
            R2_Extern.angle = 0;
            R2_Extern.speed = 0;
            R2_Extern.Area1_flag = 7;
          }
        break;

        case 7:
          R2_Extern.angle5 = 0;
          if(R2_Extern.check_angle5_flag == 1)
          {
            R2_Extern.Area1_flag = 8;
          } 
        break;

        case 8:
          // R2_Extern.Area1_dx = -(float)(visual_data.dx) * 1.0f;
          // back_keep_x(area_1_dt35[3][0]+200,0,0);
          // back_keep_y(area_1_dt35[3][1]+600,0,0);
          quzhua_dui(area_1_dt35[1][1] + 600 , -R2_Extern.Area1_dx*1.0);
          if(visual_data.usblightid == 0)//重新拿武器
          {
            R2_Extern.angle5 = 140;
            if(R2_Extern.Track_flag == 0)
              {
                R2_Extern.Area1_id += 1;
                R2_Extern.Area1_flag = 2;
              }
              else
              {
                R2_Extern.Area1_id -= 1;
                R2_Extern.Area1_flag = 2;
              }
              
          }
          if(visual_data.usblightid == 1)//松
          {
            zhuazi_open();
          }
          if(visual_data.usblightid == 2)//进2区
          {
            R2_Extern.angle_balance_target = 0;
            R2_Extern.Area1_flag = 9;
          }
        break;

        case 9:

        break;
        default:
        break;
      }
    }
    osDelay(1);
  }
  /* USER CODE END One_Area_Function */
}

/* USER CODE BEGIN Header_Two_Area_Function */
/**
* @brief Function implementing the Two_Area thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Two_Area_Function */
void Two_Area_Function(void *argument)
{
  /* USER CODE BEGIN Two_Area_Function */
  /* Infinite loop */
  for(;;)
  {
    if(visual_data.hmi_start != 1)
    {
      R2_Extern.angle = 0;
      R2_Extern.speed = 0;
      osDelay(5);
      continue;
    }

    const float (*area_2_special)[2] = (R2_Extern.Track_flag == 0) ? area_2_special_red : area_2_special_blue;

    if(visual_data.workl_mode == 2 && R2_Extern.check_1_flag == 0)
    {
        chassic_control_auto(&chassic_data, visual_data.x_map, visual_data.y_map, area_2_special[1][0], area_2_special[1][1], 1.0);
        R2_Extern.angle = chassic_data.angle;
        R2_Extern.speed = chassic_data.distance;
        check_dingwei_2(visual_data.x_map, visual_data.y_map, area_2_special[1][0], area_2_special[1][1]);
        if(R2_Extern.bool_check_1_flag == 1)
        {
            R2_Extern.angle = 0;
            R2_Extern.speed = 0;
            R2_Extern.check_1_flag = 1;
        }
    }

    if(visual_data.workl_mode == 2 && R2_Extern.check_1_flag == 1 && R2_Extern.bool_meilin_flag == 0)
    {

      
          if(R2_Extern.complete_flag == 0)
          {
            if (R2_Extern.meilin_count_flag < visual_data.meilin_count)
            {
              MeilinPointCmd current_point = visual_data.meilin_points[R2_Extern.meilin_count_flag];
              R2_Extern.KFS_Grap_flag = current_point.has_true_kfs;//取矿，暂时注释
              R2_Extern.qukuang_mode_flag = current_point.vertical_s;
              if(R2_Extern.KFS_Grap_flag == 1)
              {
                R2_Extern.bool_KFS_flag = 1;
              }
              else
              {
                R2_Extern.bool_KFS_flag = 0;
              }

              if (current_point.cell == 0)
              {
                  R2_Extern.meilin_count_flag++;
                  continue;
              }

              if (current_point.cell > 15)
              {
                  R2_Extern.angle = 0;
                  R2_Extern.speed = 0;
                  R2_Extern.meilin_count_flag++;
                  continue;
              }

              switch(current_point.horizontal_s)
              {
                case 1: R2_Extern.angle_balance_target = 90; break;
                case 2: R2_Extern.angle_balance_target = -90;  break;
                case 3: R2_Extern.angle_balance_target = 0;   break;
                case 4: R2_Extern.angle_balance_target = 180; break;
                default: break;
              }

              if (current_point.cell >= 13 && current_point.cell <= 15)
              {
                  // R2_Extern.bool_KFS_flag = 1;
                  // R2_Extern.KFS_Grap_flag = 1;
                  if (R2_Extern.chack_yaw_flag == 1)
                  {
                      R2_Extern.Area2_flag = current_point.cell;
                      R2_Extern.complete_flag = 1;
                  }
              }
              else if (R2_Extern.chack_yaw_flag == 1)
              {
                  switch(current_point.vertical_s)
                  {
                      case 5:
                      R2_Extern.Area2_flag = 1;
                      R2_Extern.pitch_angle = 0;
                      break;
                      case 6:
                      R2_Extern.Area2_flag = 5;
                      R2_Extern.pitch_angle = -10;
                      break;
                      default: break;
                  }

                  R2_Extern.complete_flag = 1;
              }
            }

          }

          if(R2_Extern.complete_taijie_flag == 1 && R2_Extern.complete_dingwei_flag == 1)
          {
            R2_Extern.complete_flag = 0;
            R2_Extern.complete_taijie_flag = 0;
            R2_Extern.complete_dingwei_flag = 0;
            R2_Extern.bool_check_1_flag = 0;
            R2_Extern.meilin_count_flag++;
            if(R2_Extern.meilin_count_flag == visual_data.meilin_count)
            {
              if(R2_Extern.complete_erqu_flag == 0)
                {
                  R2_Extern.complete_erqu_flag = 1;
                  R2_Extern.angle_balance_target = 0;
                  R2_Extern.Area2_flag = 5;
                }
            }
          }
      
      if(R2_Extern.bool_KFS_flag == 0)
      {
        switch (R2_Extern.Area2_flag)
        {
        case 0:
            R2_Extern.angle = 0;
            R2_Extern.speed = 0;
            R2_Extern.angle_balance_target = 0;
            
            if(R2_Extern.chack_yaw_flag == 1)
            {
              R2_Extern.car_flag = 1;
            }
          break;

        case 1:
            
            R2_Extern.car_flag = 0;
            // R2_Extern.lift_mood = 1;
            if(R2_Extern.lift_mood == 0)
            {
              R2_Extern.lift_mood = 1;
              osDelay(800);
            }
            if(R2_Extern.chack_yaw_flag == 1 && R2_Extern.KFS_Get_flag == 0)
            {
              R2_Extern.angle = 0;
              R2_Extern.speed = 0.5;
            }
            else
            {
              R2_Extern.angle = 0;
              R2_Extern.speed = 0;
            }
            
            if(Area_Flag.qian_dis == 1 && R2_Extern.lift_mood == 1)
            {
                R2_Extern.angle = 0;
                R2_Extern.speed = 0;
                R2_Extern.Area2_flag = 2;
            }
          break;

        case 2:
            if(R2_Extern.chsaaic_behind_flag == 0)
            {
              chsaaic_behind_up();
              R2_Extern.chsaaic_behind_flag = 1;
              osDelay(500);
            }
            R2_Extern.lift_mood = 2;
            osDelay(500);
            R2_Extern.Area2_flag = 23;
          break;

        case 23:
            now_mood.mood = 2;
            R2_Extern.angle = 0;
            R2_Extern.speed = 1.0;
            osDelay(500);
            R2_Extern.Area2_flag = 3;

        case 3:
            now_mood.mood = 1;
            R2_Extern.angle = 0;
            R2_Extern.speed = 0.3;
            if(Area_Flag.hou_dis == 1)
            {
              now_mood.mood = 0;
              R2_Extern.angle = 0;
              R2_Extern.speed = 0;
              chsaaic_behind_down();
              // R2_Extern.lift_mood = 0;
              R2_Extern.lift_mood = 1;
              R2_Extern.chsaaic_behind_flag = 0;
              osDelay(500);
              R2_Extern.Area2_flag = 4;
            }
          break;

        case 4:
            R2_Extern.angle = 0;
            R2_Extern.speed = 0.3;
            osDelay(500);
            // R2_Extern.lift_mood = 1;
            R2_Extern.Area2_flag = 0;
            R2_Extern.complete_taijie_flag = 1;

          break;
//上台阶1到4

        case 5:
            R2_Extern.car_flag = 0;
            if(R2_Extern.lift_mood == 0)
            {
              R2_Extern.angle = 0;
              R2_Extern.speed = 0;
              R2_Extern.lift_mood = 1;
              osDelay(1000);
            }
            if(R2_Extern.chack_yaw_flag == 1 && R2_Extern.KFS_Get_flag == 0)
            {
              R2_Extern.angle = 0;
              R2_Extern.speed = 0.2;
            }
            else
            {
              R2_Extern.angle = 0;
              R2_Extern.speed = 0;
            }

            if(Area_Flag.qian_dis == 2 && R2_Extern.lift_mood == 1)
            {
                R2_Extern.angle = 0;
                R2_Extern.speed = 0;
                R2_Extern.Area2_flag = 6;
            }
          break;

        case 6:
            if(R2_Extern.lift_mood == 1)
            {
              R2_Extern.lift_mood = 2;
              chsaaic_front_up();
              osDelay(1000);
            }
            R2_Extern.angle = 0;
            R2_Extern.speed = 0.2;
            now_mood.mood = 1;

            if(Area_Flag.hou_dis == 0 && R2_Extern.lift_mood == 2)
            {
              now_mood.mood = 0;
              R2_Extern.angle = 0;
              R2_Extern.speed = 0;
              osDelay(100);
              R2_Extern.Area2_flag = 7;
            }
        break;
        
        case 7:
              R2_Extern.lift_mood = 1;
              osDelay(1000);
              chsaaic_front_down();
              R2_Extern.Area2_flag = 8;

        break;

        case 8:
            R2_Extern.angle = 0;
            R2_Extern.speed = 0.2;
            osDelay(1000);
            // R2_Extern.lift_mood = 1;
            R2_Extern.Area2_flag = 0;
            R2_Extern.complete_taijie_flag = 1;
            if(R2_Extern.complete_erqu_flag == 1)
            {
              // R2_Extern.angle_balance_target = 0;
              R2_Extern.start_sanqugoon_flag = 1;
              R2_Extern.lift_mood = 0;
              R2_Extern.bool_meilin_flag = 1;
              R2_Extern.sanqugoon_step = 0;
            }
        break;

        case 13:
            chassic_control_auto(&chassic_data, visual_data.x_map, visual_data.y_map, area_2_special[0][0], area_2_special[0][1] , 1.0);
            R2_Extern.angle = chassic_data.angle;
            R2_Extern.speed = chassic_data.distance;
            check_dingwei_2(visual_data.x_map, visual_data.y_map, area_2_special[0][0], area_2_special[0][1]);
            R2_Extern.lift_mood = 1;
            if(R2_Extern.bool_check_1_flag == 1)
            {
              R2_Extern.angle = 0;
              R2_Extern.speed = 0;
              R2_Extern.lift_mood = 1;
              R2_Extern.qukuang_mode_flag = 5;
              R2_Extern.KFS_Grap_flag = 1;
              R2_Extern.bool_KFS_flag = 1;
              R2_Extern.KFS_status_flag = 1;
              R2_Extern.Area2_flag = 16;
            }
          break;

        case 14:

              // R2_Extern.complete_dingwei_flag = 1;
              // R2_Extern.complete_taijie_flag = 1;//这两行是为了让点遍历推进
              chassic_control_auto(&chassic_data, visual_data.x_map, visual_data.y_map, area_2_special[1][0], area_2_special[1][1], 1.0);
              R2_Extern.angle = chassic_data.angle;
              R2_Extern.speed = chassic_data.distance;
              check_dingwei_2(visual_data.x_map, visual_data.y_map, area_2_special[1][0], area_2_special[1][1]);
              R2_Extern.lift_mood = 0;
              if(R2_Extern.bool_check_1_flag == 1)
              {
                R2_Extern.Area2_flag = 0;
                R2_Extern.complete_dingwei_flag = 1;
                R2_Extern.complete_taijie_flag = 1;
              }
              // R2_Extern.Area2_flag = 0;
            


          break;

        case 15:
            chassic_control_auto(&chassic_data, visual_data.x_map, visual_data.y_map, area_2_special[2][0], area_2_special[2][1] , 1.0);
            R2_Extern.angle = chassic_data.angle;
            R2_Extern.speed = chassic_data.distance;
            check_dingwei_2(visual_data.x_map, visual_data.y_map, area_2_special[2][0], area_2_special[2][1]);
            R2_Extern.lift_mood = 1;
            if(R2_Extern.bool_check_1_flag == 1)
            {
              R2_Extern.angle = 0;
              R2_Extern.speed = 0;
              R2_Extern.qukuang_mode_flag = 5;
              R2_Extern.lift_mood = 1;
              R2_Extern.KFS_Grap_flag = 1;
              R2_Extern.bool_KFS_flag = 1;
              R2_Extern.KFS_status_flag = 1;
              R2_Extern.Area2_flag = 16;
            }
          break;

        case 16:
          if(R2_Extern.special_flag == 1)
          {
            R2_Extern.Area2_flag = 14;
          }
          break;

        default:
          break;
      }
    }


  }

  if(visual_data.workl_mode == 2 && R2_Extern.KFS_Grap_flag == 1 && R2_Extern.bool_KFS_flag == 1)
  {
    int x;
    if(R2_Extern.qukuang_mode_flag == 5)
    {
      x = 550;
    }
    else if(R2_Extern.qukuang_mode_flag == 6)
    {
      x = 630;
    }
      switch(R2_Extern.KFS_status_flag)
      {
        case 0:
          if(R2_Extern.Area2_flag == 13 || R2_Extern.Area2_flag == 15 || R2_Extern.Area2_flag == 16)
          {
            R2_Extern.lift_mood = 1;
          }
          else
          {
            R2_Extern.lift_mood = 0;
          }
          
          if(R2_Extern.KFS_Get_flag == 1 && visual_data.xyz_in_base[0] <= x && R2_Extern.chack_yaw_flag == 1)//上取下650 下取上570
          {
            if(R2_Extern.kfs_count_total == 3 && R2_Extern.kfs_count == 0)
            {
              R2_Extern.angle = 0;
              R2_Extern.speed = 0;
              R2_Extern.KFS_status_flag = 10;
            }
            else
            {
              R2_Extern.angle = 0;
              R2_Extern.speed = 0;
              // R2_Extern.lift_mood = 0;//降下去
              // R2_Extern.angle4 = 90;
              // R2_Extern.lift_mood = 0;
              // R2_Extern.lift_mood = 1;
              R2_Extern.KFS_status_flag = 1;
              osDelay(200);
            }
          }
          else
          {
            if(R2_Extern.chack_yaw_flag == 1)
            {
              R2_Extern.speed = 0.2;
            }
            else
            {
              R2_Extern.speed = 0;
            }
          }
        
        break;

        case 1:
        if(R2_Extern.KFS_Get_flag == 1 && visual_data.xyz_in_base[0] <= x)
        {
          qibeng_open();
          up_stair();
          if(unitree_pos[1]>= angle_3 - 10 && unitree_pos[1] <= angle_3 + 10 && dm4310_fb[1].position_deg >= angle_2 - 10 && dm4310_fb[1].position_deg <= angle_2 + 10)
          {
            // osDelay(1000);
            R2_Extern.KFS_status_flag = 3;
          }
          R2_Extern.KFS_status_flag = 3;
        }
        else
        {
          R2_Extern.speed = 0.2;
        }
        break;

        case 3:
        {
          qukuang_move_y(400);//上抓下400 下抓上400
        
          if(visual_data.kfs_x >= 380 && visual_data.kfs_x <= 420)//这也要改
          {
            R2_Extern.KFS_status_flag = 33;
          }
        }
        break;

        case 33:
            if(R2_Extern.qukuang_mode_flag == 5)
            {
              R2_Extern.angle1 = 0;
              // R2_Extern.angle2 = 120;
              R2_Extern.angle2 = 115;
              R2_Extern.angle3 = 120;//下取上
            }
            else if (R2_Extern.qukuang_mode_flag == 6)
            {
              R2_Extern.angle1 = 0;
              R2_Extern.angle2 = 110;
              // R2_Extern.angle2 = 120;
              R2_Extern.angle3 = 75;//上取下
            }

            // R2_Extern.angle4 = 80;


            // if(unitree_pos[1]>= R2_Extern.angle3 - 10 && unitree_pos[1] <= R2_Extern.angle3 + 10 && dm4310_fb[1].position_deg >= R2_Extern.angle2 - 10 && dm4310_fb[1].position_deg <= R2_Extern.angle2 + 10)
            // {
            //   // osDelay(500);
            //   // R2_Extern.angle2 = 100;
            //   // osDelay(500);
            //   // R2_Extern.angle4 = 0;
            //   R2_Extern.KFS_status_flag = 4;
            // }

            if(unitree_pos[1]>= R2_Extern.angle3 - 90 && unitree_pos[1] <= R2_Extern.angle3 + 90 && dm4310_fb[1].position_deg >= R2_Extern.angle2 - 90 && dm4310_fb[1].position_deg <= R2_Extern.angle2 + 90)
            {
                if(R2_Extern.kfs_count == 0 && R2_Extern.kfs_count_total != 3)
                {
                  R2_Extern.angle4 = 80;
                }
                else if(R2_Extern.kfs_count == 1 && R2_Extern.kfs_count_total == 3)
                {
                  R2_Extern.angle4 = 80;
                }
                else
                {
                  R2_Extern.angle4 =-20;
                }
                if(unitree_pos[1]>= R2_Extern.angle3 - 5 && unitree_pos[1] <= R2_Extern.angle3 + 5 && dm4310_fb[1].position_deg >= R2_Extern.angle2 - 5 && dm4310_fb[1].position_deg <= R2_Extern.angle2 + 5)
                {
                  R2_Extern.KFS_status_flag = 34;
                  // osDelay(500);
                  // R2_Extern.angle2 = 60;
                  // R2_Extern.angle3 = 60;
                  // if(unitree_pos[1]>= R2_Extern.angle3 - 5 && unitree_pos[1] <= R2_Extern.angle3 + 5 && dm4310_fb[1].position_deg >= R2_Extern.angle2 - 5 && dm4310_fb[1].position_deg <= R2_Extern.angle2 + 5)
        
                  // {
                  //   R2_Extern.KFS_status_flag = 4;
                  // }
                  
                }
            }
        break;

        case 34:
                  R2_Extern.angle2 = 60;
                  R2_Extern.angle3 = 100;
                  if(unitree_pos[1]>= R2_Extern.angle3 - 5 && unitree_pos[1] <= R2_Extern.angle3 + 5 && dm4310_fb[1].position_deg >= R2_Extern.angle2 - 5 && dm4310_fb[1].position_deg <= R2_Extern.angle2 + 5)
        
                  {
                    R2_Extern.KFS_status_flag = 4;
                  }
        break;
        case 4:
        R2_Extern.angle2 = 0;
        R2_Extern.angle3 = 120;
        if(unitree_pos[1]>= R2_Extern.angle3 - 10 && unitree_pos[1] <= R2_Extern.angle3 + 10 && dm4310_fb[1].position_deg >= R2_Extern.angle2 - 10 && dm4310_fb[1].position_deg <= R2_Extern.angle2 + 10)
        {
          R2_Extern.angle4 = 0;
          if(unitree_pos[1]>= R2_Extern.angle3 - 5 && unitree_pos[1] <= R2_Extern.angle3 + 5 && dm4310_fb[1].position_deg >= R2_Extern.angle2 - 5 && dm4310_fb[1].position_deg <= R2_Extern.angle2 + 50)
          {
            // osDelay(1000);
            if(R2_Extern.KFS_Get_flag == 1)
            {
              osDelay(1000);
              if(R2_Extern.KFS_Get_flag == 1)
              {
                R2_Extern.KFS_status_flag = 0;
                // R2_Extern.KFS_status_flag = 1;
              }
            }
            else
            {
              R2_Extern.KFS_status_flag = 5;
              // R2_Extern.angle4 = -90;
              // R2_Extern.special_flag = 1;
              // R2_Extern.lift_mood = 0;
              // R2_Extern.angle2 = 0;
              // R2_Extern.angle3 = -10;
              // R2_Extern.angle4 = -90;
              // R2_Extern.bool_KFS_flag = 0;
              // R2_Extern.car_flag = 0;
              // R2_Extern.kfs_count ++;
              // R2_Extern.KFS_status_flag = 0;
            
            }
          }
        }

        case 5:
                R2_Extern.special_flag = 1;
                R2_Extern.lift_mood = 0;
                R2_Extern.angle2 = 0;
                R2_Extern.angle3 = 0;
                R2_Extern.angle4 = -135;
                R2_Extern.bool_KFS_flag = 0;
                R2_Extern.car_flag = 0;
                R2_Extern.kfs_count ++;
                R2_Extern.KFS_status_flag = 0;

        break;

        
        break;

        case 10:
        R2_Extern.angle1 = 0;
        R2_Extern.angle2 = 50;
        R2_Extern.angle3 = 110;
        R2_Extern.angle4 = -50;
        if(unitree_pos[1]>= R2_Extern.angle3 - 5 && unitree_pos[1] <= R2_Extern.angle3 + 5 && dm4310_fb[1].position_deg >= R2_Extern.angle2 - 5 && dm4310_fb[1].position_deg <= R2_Extern.angle2 + 5)
        {
          fangkuang_close_2();
          osDelay(500);
          R2_Extern.kfs_count ++;
          R2_Extern.KFS_status_flag = 11;
          
        }
        break;

        case 11:
          R2_Extern.angle1 = 0;
          R2_Extern.angle2 = 10;
          R2_Extern.angle3 = 0;
          R2_Extern.angle4 = -90;
          fangkuang_open_2();
          if(unitree_pos[1]>= R2_Extern.angle3 - 5 && unitree_pos[1] <= R2_Extern.angle3 + 5 && dm4310_fb[1].position_deg >= R2_Extern.angle2 - 5 && dm4310_fb[1].position_deg <= R2_Extern.angle2 + 5)
          {
            R2_Extern.KFS_status_flag = 0;
          }
        break;

        default:
        break;

      }
    
  }

    osDelay(5);
  }
  /* USER CODE END Two_Area_Function */
}

/* USER CODE BEGIN Header_Three_Area_Function */
/**
* @brief Function implementing the Three_Area thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Three_Area_Function */
void Three_Area_Function(void *argument)
{
  /* USER CODE BEGIN Three_Area_Function */
  /* Infinite loop */
  for(;;)
  {
    if(visual_data.hmi_start != 1)
    {
      R2_Extern.angle = 0;
      R2_Extern.speed = 0;
      osDelay(5);
      continue;
    }

    const float (*area_3_dt35)[2] = (R2_Extern.Track_flag == 0) ? area_3_dt35_red : area_3_dt35_blue;
    if(R2_Extern.work_mode == 3)//暂时
    {
      switch (R2_Extern.Area3_step)
      {
      case 0:
      R2_Extern.angle_balance_target =(R2_Extern.Track_flag == 0) ? 90 : -90;
      // R2_Extern.angle1 = 0;
      // R2_Extern.angle2 = 60;
      // R2_Extern.angle3 = 120;
      // R2_Extern.angle4 = 0;
      // R2_Extern.pitch_angle = 10;
      if(R2_Extern.chack_yaw_flag == 1)
      {
        R2_Extern.Area3_step = 1;
        osDelay(500);
      }
        
        break;
        
      case 1:
      if(R2_Extern.Area3_flag == 1)
        {
          R2_Extern.Area3_step = 2;
        }
      break;
      case 2:
        // R2_Extern.Area3_cell = visual_data.target_cell - 3;
        R2_Extern.Area3_cell = 1;
        R2_Extern.angle2 = 60;
        R2_Extern.angle3 = 90;
        if(R2_Extern.kfs_count_total == 2)
        {
          R2_Extern.angle4 = -130;
        }
        else
        {
          R2_Extern.angle4 = -40;
        }
        R2_Extern.Area3_step = 3;
        break;
      case 3:
        fangkuang(area_3_dt35[R2_Extern.Area3_cell][0], area_3_dt35[R2_Extern.Area3_cell][1]);
        if(R2_Extern.Area3_2_flag == 1)
        {
          R2_Extern.Area3_step = 4;
        }
        break;
        
      case 4:
//放的时候先放下面的，再放上面的
        fangkuang_close_1();
        fangkuang_close_2();

        break;
      
      default:
        break;
      }

    }
    osDelay(1);
  }
  /* USER CODE END Three_Area_Function */
}

/* USER CODE BEGIN Header_Mid360_Function */
/**
* @brief Function implementing the Mid360 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Mid360_Function */
void Mid360_Function(void *argument)
{
  /* USER CODE BEGIN Mid360_Function */

  /* Infinite loop */
  for(;;)
  {
    if(visual_data.hmi_start != 1)
    {
      R2_Extern.angle = 0;
      R2_Extern.speed = 0;
      osDelay(5);
      continue;
    }

    const float (*area_1)[2] = (R2_Extern.Track_flag == 0) ? area_1_red : area_1_blue;
    const float (*area_2)[2] = (R2_Extern.Track_flag == 0) ? area_2_red : area_2_blue;
    const float (*area_3)[2] = (R2_Extern.Track_flag == 0) ? area_3_red : area_3_blue;
    const float (*data_table)[2] = (R2_Extern.Track_flag == 0) ? data_table_red : data_table_blue;

    if(visual_data.i == 1)
    {
      if (visual_data.workl_mode == 1)
      {
        switch(R2_Extern.Area1_step)
        {
          case 0:
          if(R2_Extern.Area1_1_flag == 0)
          {
            chassic_control_auto(&chassic_data, visual_data.x_map, visual_data.y_map, area_1[0][0], area_1[0][1] , 1.0);
            R2_Extern.angle = chassic_data.angle;
            R2_Extern.speed = chassic_data.distance;
            check_dingwei_2(visual_data.x_map, visual_data.y_map, area_1[0][0], area_1[0][1]);
            if(R2_Extern.bool_check_1_flag == 1)
            {
              R2_Extern.angle = 0;
              R2_Extern.speed = 0;
              R2_Extern.Area1_1_flag =1;
              R2_Extern.Area1_step = 1;
            }
          }
          break;

          case 1:
          break;

          default:
          break;
        }
       
      }


      if(visual_data.workl_mode == 2)
      {
        if(R2_Extern.complete_taijie_flag == 1 && R2_Extern.complete_dingwei_flag == 0 && R2_Extern.start_sanqugoon_flag == 0 && R2_Extern.chack_yaw_flag == 1)
        {
          uint8_t current_cell = visual_data.meilin_points[R2_Extern.meilin_count_flag].cell;
          if(current_cell >= 1 && current_cell <= 12)
          {
            chassic_control_auto(&chassic_data, visual_data.x_map, visual_data.y_map, data_table[current_cell-1][0], data_table[current_cell-1][1] , 2.0);
            R2_Extern.angle = chassic_data.angle;
            R2_Extern.speed = chassic_data.distance;
          }
          else
          {
              R2_Extern.angle = 0;
              R2_Extern.speed = 0;
          }
        }
        else if(R2_Extern.start_sanqugoon_flag == 1)
        {
          // 三圈果定位：依次经过两个中间点
          float dx1 = visual_data.x_map - area_2[0][0];
          float dy1 = visual_data.y_map - area_2[0][1];
          float dist1 = sqrtf(dx1 * dx1 + dy1 * dy1);

          float dx2 = visual_data.x_map - area_2[1][0];
          float dy2 = visual_data.y_map - area_2[1][1];
          float dist2 = sqrtf(dx2 * dx2 + dy2 * dy2);

          switch(R2_Extern.sanqugoon_step)
          {
            case 0: 
              if(dist1 <= 0.10f)
              {
                R2_Extern.sanqugoon_step = 1; // 切换到点2
              }
              else
              {
                chassic_control_auto(&chassic_data, visual_data.x_map, visual_data.y_map, area_2[0][0], area_2[0][1] , 1.0);
                R2_Extern.angle = chassic_data.angle;
                R2_Extern.speed = chassic_data.distance;
              }
              break;

            case 1: // 走向点2 (11.02, -4.24)
              if(dist2 <= 0.10f)
              {
                R2_Extern.complete_dingwei_flag = 1;
                R2_Extern.angle = 0;
                R2_Extern.speed = 0;
                R2_Extern.work_mode = 3;
              }
              else
              {
                chassic_control_auto(&chassic_data, visual_data.x_map, visual_data.y_map, area_2[1][0], area_2[1][1] , 1.5);
                R2_Extern.angle = chassic_data.angle;
                R2_Extern.speed = chassic_data.distance;
              }
              break;

            default:
              break;
          }
        }
      }


      if(visual_data.workl_mode == 3)
      {
        if(R2_Extern.Area3_flag == 0 && R2_Extern.Area3_step == 1)
        {
          chassic_control_auto(&chassic_data, visual_data.x_map, visual_data.y_map, area_3[0][0], area_3[0][1] , 1.0);
          R2_Extern.angle = chassic_data.angle;
          R2_Extern.speed = chassic_data.distance;
          check_dingwei_2(visual_data.x_map, visual_data.y_map, area_3[0][0], area_3[0][1]);
          {
            if(R2_Extern.bool_check_1_flag == 1)
            {
              R2_Extern.Area3_flag =1;
            }
          }
        }
      }

    }

    osDelay(5);
  }
  /* USER CODE END Mid360_Function */
}

/* USER CODE BEGIN Header_Track_Mode_Function */
/**
* @brief Function implementing the Track_Mode thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Track_Mode_Function */
void Track_Mode_Function(void *argument)
{
  /* USER CODE BEGIN Track_Mode_Function */
  /* Infinite loop */
  for(;;)
  {
    if(visual_data.hmi_color == 1)
    {
      R2_Extern.Track_flag = 0;//红
      RGB_Color_Ctrl(1,255,1);//绿
    }
    else if (visual_data.hmi_color == 2)
    {
      R2_Extern.Track_flag = 1;//蓝
      RGB_Color_Ctrl(255,255,1);//黄
    }
    osDelay(10);
  }
  /* USER CODE END Track_Mode_Function */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

