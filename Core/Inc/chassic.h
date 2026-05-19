//written by Fred Xiao

#ifndef __CHASSIC_H
#define __CHASSIC_H
#include "main.h"



#include <stdint.h>

#define MAX_PWM_VALUE 16384.0f

// 调试符号系数（如果运动方向不对，调整这些系数）
// 默认值为 1.0f，如果方向相反改为 -1.0f
#define COEFF_VX  1.0f
#define COEFF_VY  1.0f
#define COEFF_W   -1.0f

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

    Motor motor5;
    Motor motor6;
    Motor motor7;
    Motor motor8;

    int mood;
} Chassic;

// 全局变量声明
extern Chassic result;


void chassic_control(float angle_deg, float speed_mag, float w_rotation);
void chassic_small(void);

#endif 
