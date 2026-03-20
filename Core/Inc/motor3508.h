#ifndef __MOTOR3508_H
#define __MOTOR3508_H
#include "main.h"

typedef struct
{
    int   NowSpeed;				//现在速度
    int   TargetSpeed;			//目标速度
	int   filter_TargetSpeed;   //滤波后目标速度

    int   NowEnconder;			//现在编码器的值
    int   LastEnconder;			//上次编码器的值
	int   target_distance;		//目标距离
	int   accumulated_distance;	//累计距离



    float ExpectAngle;			//期望角度 可通过脉冲数转换
    float OutSpeed;				//PID控制器输出

} DJmotor_struct;      			//DJI电机控制


extern DJmotor_struct motor[8];

// FDCAN发送函数
void FDCAN_cmd_chassis_fdcan1_0x200(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void FDCAN_cmd_chassis_fdcan1_0x1FF(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void FDCAN_cmd_chassis_fdcan2_0x200(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void FDCAN_cmd_chassis_fdcan2_0x1FF(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

// 电机数据处理函数
void Motor3508_Process_Rx_Message(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData);

#endif

