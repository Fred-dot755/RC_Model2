//written by Fred Xiao

#include "control.h"
#include <stdio.h>

My_extern R2_Extern;
Area2_control_t Area_Flag;

StairState_t current_state = STATE_SIX;


chassic_control_t chassic_data;

void chassic_control_auto(chassic_control_t *chassic_data, float now_x, float now_y, float target_x, float target_y)
{
    chassic_data->now_x = now_x;
    chassic_data->now_y = now_y;
    chassic_data->target_x = target_x;
    chassic_data->target_y = target_y;

    double dx = (double)(target_x - now_x);
    double dy = (double)(target_y - now_y);
    double real_distance = sqrt(dx * dx + dy * dy);

    int raw_angle = (int)(atan2(dy, dx) * 180.0 / PI);
     if (raw_angle > 180) {
        raw_angle -= 360;
    }
    chassic_data->angle = -raw_angle;
    
    float MAX_SPEED = 2.0; 
    double ONE_METER = 0.5;
    double DEADZONE = 0.001;

    if (real_distance < DEADZONE) 
    {
        chassic_data->distance = 0; 
    }
    else if (real_distance >= ONE_METER) 
    {
        chassic_data->distance = MAX_SPEED; 
    }
    else 
    {
        chassic_data->distance = 4 * real_distance;
    }
}



//1区控制
void zhuazi_mode_1_5(void)
{
    set_quzhua_angle(180);
}
void zhuazi_mode_2(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
}
void zhuazi_mode_3(void)
{
    set_quzhua_angle(265);
}
void zhuazi_mode_4(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
}
void zhuazi_mode_6(void)
{
    motor[10].ExpectAngle = 474552;
}
void zhuazi_mode_7(void)
{
    set_quzhua_angle(0);
}



//2区控制
void chassic_up(void)//整体上升
{
    R2_Extern.lift_mood = 1;
}

void chassic_down(void)//整体下降
{
    R2_Extern.lift_mood = 0;
}

void chsaaic_behind_up(void)//后轮下降
{
    // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
}

void chsaaic_behind_down(void)//后轮上升
{
    // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
}

void chsaaic_front_up(void)//前轮下降
{
    // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
}

void chsaaic_front_down(void)//前轮上升
{
    // HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
}

void grap_on(void)//打开
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
}

void grap_off(void)//关闭
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

#define angle3508_to_deg 8198 * 19 / 360
void grab_angle(int angle)
{
    motor[10].ExpectAngle = angle3508_to_deg * angle;
}










const float data_table[12][2] = {
    {3.27f, -0.47f},//1
    {3.27f, -1.64f},//2
    {3.28f, -2.85f},//3
    {4.50f, -0.50f},//4
    {4.50f, -1.66f},//5
    {4.47f, -2.88f},//6
    {5.67f, -0.50f},//7
    {5.71f, -1.72f},//8
    {5.71f, -2.87f},//9
    {6.94f, -0.60f},//10
    {6.90f, -1.78f},//11
    {6.89f, -2.95f}//12
};

void check_dingwei(float current_x, float current_y, int cell_index)
{
    // cell_index: 0~11，对应 data_table[0]~data_table[11]
    if (cell_index < 0 || cell_index >= 12)
    {
        R2_Extern.complete_dingwei_flag = 0;
        return;
    }

    float dx = current_x - data_table[cell_index][0];
    float dy = current_y - data_table[cell_index][1];
    float dist = sqrtf(dx * dx + dy * dy);

    R2_Extern.complete_dingwei_flag = (dist <= 0.05f) ? 1 : 0;
}

void check_dingwei_2(float current_x, float current_y, float target_x, float target_y)
{
    float dx = current_x - target_x;
    float dy = current_y - target_y;
    float dist = sqrtf(dx * dx + dy * dy);

    R2_Extern.bool_check_1_flag = (dist <= 0.07f) ? 1 : 0;
}

//3区控制


//

float v[4];
void v_update(void)
{
    v[0] = -(float)motor[0].NowSpeed * CUR_TO_RPM;
    v[1] = -(float)motor[1].NowSpeed * CUR_TO_RPM;
    v[2] = -(float)motor[2].NowSpeed * CUR_TO_RPM;
    v[3] = -(float)motor[3].NowSpeed * CUR_TO_RPM;
}