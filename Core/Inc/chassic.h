#ifndef __CHASSIC_H
#define __CHASSIC_H
#include "main.h"



#include <stdint.h>

#define MAX_PWM_VALUE 8000.0f 

// 定义圆周率
#ifndef PI
#define PI 3.1415926535f    
#endif

typedef struct {
    int16_t out;
} Motor;

typedef struct {
    Motor motor1; // 左前 (FL)
    Motor motor2; // 右前 (FR)
    Motor motor3; // 右后 (RR)
    Motor motor4; // 左后 (RL)
} Chassic;

// 全局变量声明
extern Chassic result;


void chassic_control(float angle_deg, float speed_mag, float w_rotation);


#endif 
