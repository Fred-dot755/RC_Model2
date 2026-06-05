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
    set_yuntai_angle(0,0);
    meic_protocol_send_packet_dma(&huart10, 0, 0);

    R2_Extern.lingshi_flag = unitree_pos[1] - unitree_pos[2];

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
    //1负3正，2左正右负
    unitree_cmd_create(&unitree_cmd[1], 1, 1, 5.5, 0.3, now_position.output_angle2, 0.0, 0.0);
    unitree_communicate(1);
    osDelay(10);
    unitree_cmd_create(&unitree_cmd[3], 3, 1, 5.5, 0.3, -now_position.output_angle2, 0.0, 0.0);
    unitree_communicate(3);
    osDelay(10);
    unitree_cmd_create(&unitree_cmd[2], 2, 1, 1.0, 0.2, now_position.output_angle1, 0.0, 0.0);
    unitree_communicate(2);
    osDelay(10);

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

    DM_CAN_Send_PosVel_Mode(-(-R2_Extern.angle4  + unitree_pos[1] - dm4310_fb[1].position_deg) * 1.5,80,2);//上正
    DM_CAN_Send_PosVel_Mode(-R2_Extern.angle3,60,3);//上负
    DM_CAN_Send_PosVel_Mode(-R2_Extern.lift,400,4);
    DM_CAN_Send_PosVel_Mode(-R2_Extern.lift,400,5);
  
    //调试用
    // DM_CAN_Send_PosVel_Mode(0,0,2);
    // DM_CAN_Send_PosVel_Mode(0,0,3);
    // DM_CAN_Send_PosVel_Mode(0,0,4);
    // DM_CAN_Send_PosVel_Mode(0,0,5);
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
    grab_angle(R2_Extern.angle5);

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
    R2_Extern.angle = rc_data.angle;
    R2_Extern.speed = rc_data.distance *1;
    // if(R2_Extern.get_init == 0)
    // {
    //   R2_Extern.angle2 = angle_2;
    //   R2_Extern.angle3 = angle_3;
    //   // R2_Extern.lift_mood = 1;
    //   // osDelay(1000);
    //   // chsaaic_behind_up();
    //   // chsaaic_front_up();
    //   // R2_Extern.angle2 = 145;
    //   // R2_Extern.angle3 = 80;
    //   // R2_Extern.angle4 = 0;
    //   if(unitree_pos[1]>= R2_Extern.angle2 - 5 && unitree_pos[1] <= R2_Extern.angle2 + 5 && dm4310_fb[1].position_deg >= R2_Extern.angle3 - 5 && dm4310_fb[1].position_deg <= R2_Extern.angle3 + 5)
    //   {
    //     R2_Extern.get_init = 1;
    //   }
    // }


    if(rc_data.btn_1 == 1)
    {
      R2_Extern.angle_balance_target += 1;
    }
    if(rc_data.btn_2 == 1)
    {
      R2_Extern.angle_balance_target -= 1;
    }
    if(rc_data.btn_3 == 1)
    {
      R2_Extern.Area2_flag = 1;
    }
    if(rc_data.btn_4 == 1)
    {
      R2_Extern.Area2_flag = 5;
    }

    //     if(rc_data.btn_1 == 1 && rc_data.btn_2 == 1)
    // {
    //   R2_Extern.angle5 = -80;
    //   osDelay(100);
    // }
    // else if(rc_data.btn_1 == 1 && rc_data.btn_3 == 1)
    // {
    //   R2_Extern.angle5 = 0;
    //   osDelay(100);
    // }
    // else if(rc_data.btn_2 == 1 && rc_data.btn_3 == 1)
    // {
    //   grap_on();
    //   osDelay(100);
    // }
    // else if(rc_data.btn_2 == 1 && rc_data.btn_4 == 1)
    // {
    //   grap_off();
    //   osDelay(100);
    // }
    // else if(rc_data.btn_3 == 1 && rc_data.btn_4 == 1)
    // {
    //   R2_Extern.angle2 = 0;
    //   osDelay(1000);
    //   R2_Extern.angle3 = 80;
    //   R2_Extern.angle4 = 0;
    // }

    
      switch (R2_Extern.Area2_flag)
      {
      case 0:
          // R2_Extern.angle = 0;
          // R2_Extern.speed = 0;
          // R2_Extern.angle_balance_target = 0;
        break;

      case 1:
          if(R2_Extern.lift_mood == 0)
          {
            R2_Extern.lift_mood = 1;
            osDelay(2000);
          }
          R2_Extern.angle = 0;
          R2_Extern.speed = 0.5;
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
          osDelay(2000);
          R2_Extern.Area2_flag = 3;
        break;

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
            R2_Extern.lift_mood = 0;
            R2_Extern.chsaaic_behind_flag = 0;
            osDelay(1000);
            R2_Extern.Area2_flag = 4;
          }
        break;

      case 4:
          R2_Extern.angle = 0;
          R2_Extern.speed = 0.5;
          osDelay(500);
          R2_Extern.Area2_flag = 0;
          R2_Extern.complete_taijie_flag = 1;

        break;
//上台阶1到4

      case 5:
          if(R2_Extern.lift_mood == 0)
          {
            R2_Extern.lift_mood = 1;
            osDelay(1000);
          }
          if(R2_Extern.chack_yaw_flag == 1)
          {
            R2_Extern.angle = 0;
            R2_Extern.speed = 0.15;
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
            osDelay(2500);
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
            osDelay(2000);
            chsaaic_front_down();
            R2_Extern.Area2_flag = 8;

      break;

      case 8:
          R2_Extern.angle = 0;
          R2_Extern.speed = 0.3;
          osDelay(1000);
          R2_Extern.lift_mood = 0;
          R2_Extern.Area2_flag = 0;
          R2_Extern.complete_taijie_flag = 1;
          if(R2_Extern.complete_erqu_flag == 1)
          {
            R2_Extern.start_sanqugoon_flag = 1;
            R2_Extern.bool_meilin_flag = 1;
          }
      break;
//下台阶5到8

      default:
        break;
    }




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
    computeDHTransform(unitree_pos[0], unitree_pos[1],dm4310_fb[1].position_deg , &R2_Extern.x_t, &R2_Extern.y_t, &R2_Extern.z_t);
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
    else if(L1_Sensor1.distance_mm >= 500)
    {
      Area_Flag.qian_dis = 2;//高
    }
    else
    {
      Area_Flag.qian_dis = 0;
    }

    if(L1_Sensor2.distance_mm <= 180)
    {
      Area_Flag.hou_dis = 1;//低
    }
    else if(L1_Sensor2.distance_mm >= 500)
    {
      Area_Flag.hou_dis = 2;//高
    }
    else
    {
      Area_Flag.hou_dis = 0;
    }


    if(abs(ops.HIPNUCaccelerationZ - R2_Extern.angle_balance) < 10)
    {
      R2_Extern.chack_yaw_flag = 1;
    }
    else
    {
      R2_Extern.chack_yaw_flag = 0;
    }


    check_dingwei(visual_data.x_map, visual_data.y_map, visual_data.meilin_points[R2_Extern.meilin_count_flag].cell - 1);


    if(visual_data.bool_getKFS == 1)
    {
      R2_Extern.KFS_Get_flag = 1;
    }
    else
    {
      R2_Extern.KFS_Get_flag = 0;
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
    arm_unitree_planning_update(R2_Extern.angle1, R2_Extern.angle2);
    osDelay(50);
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

    // === angle_balance 逐度逼近 angle_balance_target（类似 arm_unitree_planning_update 的逻辑）===
    float balance_diff = R2_Extern.angle_balance_target - R2_Extern.angle_balance;
    // 将差值归一化到 [-180, 180]，确保以劣弧方向旋转
    while (balance_diff > 180.0f) balance_diff -= 360.0f;
    while (balance_diff < -180.0f) balance_diff += 360.0f;

    if (balance_diff > 0.3f) {
        R2_Extern.angle_balance += 0.3f;
    } else if (balance_diff < -0.3f) {
        R2_Extern.angle_balance -= 0.3f;
    } else {
        R2_Extern.angle_balance = R2_Extern.angle_balance_target;  // 差值小于 1 度，直接到位
    }

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
      R2_Extern.lift = 490;
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
      R2_Extern.lift = -30;
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
//     if(visual_data.workl_mode == 2 && R2_Extern.check_1_flag == 0)
//     {
//         chassic_control_auto(&chassic_data, visual_data.x_map, visual_data.y_map, 2.22, -1.50);
//         R2_Extern.angle = chassic_data.angle;
//         R2_Extern.speed = chassic_data.distance;
//         check_dingwei_2(visual_data.x_map, visual_data.y_map, 2.22, -1.50);
//         if(R2_Extern.bool_check_1_flag == 1)
//         {
//             R2_Extern.angle = 0;
//             R2_Extern.speed = 0;
//             R2_Extern.check_1_flag = 1;
//         }
//     }

//     if(visual_data.workl_mode == 2 && R2_Extern.KFS_Get_flag == 0 && R2_Extern.check_1_flag == 1 && R2_Extern.bool_meilin_flag == 0)
//     {

      
//           if(R2_Extern.complete_flag == 0)
//           {
//             if (R2_Extern.meilin_count_flag < visual_data.meilin_count)
//             {
//               MeilinPointCmd current_point = visual_data.meilin_points[R2_Extern.meilin_count_flag];

//               if (current_point.cell == 0)
//               {
//                   R2_Extern.meilin_count_flag++; 
//                   return;
//               }

//               switch(current_point.horizontal_s)
//               {
//                 case 1: R2_Extern.angle_balance_target = 90; break;
//                 case 2: R2_Extern.angle_balance_target = -90;  break;
//                 case 3: R2_Extern.angle_balance_target = 0;   break;
//                 case 4: R2_Extern.angle_balance_target = 180; break;
//                 default: break;
//               }

//               if (R2_Extern.chack_yaw_flag == 1)
//               {
//                   switch(current_point.vertical_s)
//                   {
//                       case 5: R2_Extern.Area2_flag = 1; break;
//                       case 6: R2_Extern.Area2_flag = 5; break;
//                       default: break;
//                   }

//                   R2_Extern.complete_flag = 1;
//               }
//             }

//           }

//           if(R2_Extern.complete_taijie_flag == 1 && R2_Extern.complete_dingwei_flag == 1)
//           {
//             R2_Extern.complete_flag = 0;
//             R2_Extern.complete_taijie_flag = 0;
//             R2_Extern.meilin_count_flag++;
//             if(R2_Extern.meilin_count_flag == visual_data.meilin_count)
//             {
//               if(R2_Extern.complete_erqu_flag == 0)
//                 {
//                   R2_Extern.complete_erqu_flag = 1;
//                   R2_Extern.Area2_flag = 5;
//                 }
//             }
//           }
      
//       switch (R2_Extern.Area2_flag)
//       {
//       case 0:
//           R2_Extern.angle = 0;
//           R2_Extern.speed = 0;
//           R2_Extern.angle_balance_target = 0;
//         break;

//       case 1:
//           if(R2_Extern.lift_mood == 0)
//           {
//             R2_Extern.lift_mood = 1;
//             osDelay(2000);
//           }
//           R2_Extern.angle = 0;
//           R2_Extern.speed = 0.5;
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
//           R2_Extern.speed = 0.5;
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
//             R2_Extern.speed = 0.15;
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


//   }

  if(visual_data.workl_mode == 2 && R2_Extern.get_init == 1)
  {
    switch(R2_Extern.KFS_status_flag)
    {
      case 0:
      if(R2_Extern.KFS_Get_flag == 1)
      {
        R2_Extern.angle = 0;
        R2_Extern.speed = 0;
        R2_Extern.lift_mood = 0;
        R2_Extern.angle4 = 80;
        R2_Extern.KFS_status_flag = 1;
        osDelay(200);
      }
      break;

      case 2:
      up_stair();
      R2_Extern.KFS_status_flag = 3;
      break;

      case 3:
      inverseKinematics(R2_Extern.x, R2_Extern.y, R2_Extern.z,&R2_Extern.angle1, &R2_Extern.angle2, &R2_Extern.angle3);
      if(unitree_pos[1]>= R2_Extern.angle2 - 5 && unitree_pos[1] <= R2_Extern.angle2 + 5 && dm4310_fb[1].position_deg >= R2_Extern.angle3 - 5 && dm4310_fb[1].position_deg <= R2_Extern.angle3 + 5)
      {
        osDelay(500);
        R2_Extern.KFS_status_flag = 4;
      }
      break;

      case 4:
      R2_Extern.angle2 = angle_2;
      R2_Extern.angle3 = angle_3;
      R2_Extern.angle4 = -10;
      if(unitree_pos[1]>= R2_Extern.angle2 - 5 && unitree_pos[1] <= R2_Extern.angle2 + 5 && dm4310_fb[1].position_deg >= R2_Extern.angle3 - 5 && dm4310_fb[1].position_deg <= R2_Extern.angle3 + 5)
      {
        osDelay(500);
        if(R2_Extern.KFS_Get_flag == 1)
        {
          osDelay(1000);
          if(R2_Extern.KFS_Get_flag == 1)
          {
            R2_Extern.KFS_status_flag = 0;
          }
        }
        else
        {
          R2_Extern.KFS_status_flag = 5;
        }
      }
      break;

      case 5:

        R2_Extern.angle1 = 155;
        R2_Extern.angle2 = angle_2;
        R2_Extern.angle4 = 80;
        if(unitree_pos[0] >= R2_Extern.angle1 - 5 && R2_Extern.angle1 <= unitree_pos[0] + 5)
        {
            R2_Extern.angle2 = angle_2 + 20;
            R2_Extern.angle3 = angle_3 + 20;
            if(unitree_pos[1]>= R2_Extern.angle2 - 5 && unitree_pos[1] <= R2_Extern.angle2 + 5 && dm4310_fb[1].position_deg >= R2_Extern.angle3 - 5 && dm4310_fb[1].position_deg <= R2_Extern.angle3 + 5)
            {
              R2_Extern.angle2 = angle_2 + 30;
              R2_Extern.angle3 = angle_3 + 30;
              R2_Extern.KFS_status_flag = 0;
              osDelay(500);
            }
            
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
  static BSTracker bs_tracker;
  static uint8_t   bs_path_planned = 0;

  /* Infinite loop */
  for(;;)
  {
    if(visual_data.i == 1)
    {
      if (visual_data.workl_mode == 1)
      {
          if (!bs_path_planned)
          {
              BSPath path;
              bspline_plan_path(visual_data.x_map, visual_data.y_map,
                                chassic_data.target_x, chassic_data.target_y,
                                NULL, NULL, 0,
                                &path, BSPLINE_PATH_SAMPLES);
              bspline_tracker_init(&bs_tracker, &path,
                                  visual_data.x_map, visual_data.y_map,
                                  chassic_data.target_x, chassic_data.target_y);
              bs_path_planned = 1;
          }

          float next_x, next_y;
          int ret = bspline_tracker_next(&bs_tracker,
                                        visual_data.x_map, visual_data.y_map,
                                        &next_x, &next_y);
          if (ret == 1)
          {
              chassic_control_auto(&chassic_data,
                                  visual_data.x_map, visual_data.y_map,
                                  next_x, next_y);
          }
          else
          {
              bs_path_planned = 0;
          }
      }
      else
      {
          bs_path_planned = 0;
          bspline_tracker_reset(&bs_tracker);
      }


      if(visual_data.workl_mode == 2)
      {
        if(R2_Extern.complete_taijie_flag == 1 && R2_Extern.complete_dingwei_flag == 0)
        {
          chassic_control_auto(&chassic_data, visual_data.x_map, visual_data.y_map, data_table[visual_data.meilin_points[R2_Extern.meilin_count_flag].cell-1][0], data_table[visual_data.meilin_points[R2_Extern.meilin_count_flag].cell-1][1]);
          R2_Extern.angle = chassic_data.angle;
          R2_Extern.speed = chassic_data.distance;
        }

        if(R2_Extern.start_sanqugoon_flag == 1)
        {
          float dx1 = visual_data.x_map - 8.40f;
          float dy1 = visual_data.y_map - (-3.60f);
          float dist1 = sqrtf(dx1 * dx1 + dy1 * dy1);

          float dx2 = visual_data.x_map - 11.17f;
          float dy2 = visual_data.y_map - (-3.60f);
          float dist2 = sqrtf(dx2 * dx2 + dy2 * dy2);

          if(dist2 <= 0.05f)
          {
            R2_Extern.complete_dingwei_flag = 1;
            R2_Extern.angle = 0;
            R2_Extern.speed = 0;
          }
          else if(dist1 <= 0.05f)
          {
            chassic_control_auto(&chassic_data, visual_data.x_map, visual_data.y_map, 11.17, -3.60);
            R2_Extern.angle = chassic_data.angle;
            R2_Extern.speed = chassic_data.distance;
          }
          else
          {
            chassic_control_auto(&chassic_data, visual_data.x_map, visual_data.y_map, 8.40, -3.60);
            R2_Extern.angle = chassic_data.angle;
            R2_Extern.speed = chassic_data.distance;
          }
        }
      }


      if(visual_data.workl_mode == 3)
      {
        
      }

    }

    osDelay(5);
  }
  /* USER CODE END Mid360_Function */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

