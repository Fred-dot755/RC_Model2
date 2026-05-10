//written by Fred Xiao

#include "control.h"
#include <stdio.h>

My_extern R2_Extern;


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
    
    int MAX_SPEED = 2000; 
    double ONE_METER = 0.2;
    double DEADZONE = 0.05;

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
        chassic_data->distance = (int)(MAX_SPEED * real_distance);
    }
}



//1区控制



//2区控制
void chassic_up(void)//整体上升
{
    R2_Extern.lift_mood = 1;
}

void chassic_down(void)//整体下降
{
    R2_Extern.lift_mood = 0;
}

void chsaaic_behind_up(void)//后轮上升
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
}

void chsaaic_behind_down(void)//后轮下降
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
}

void chsaaic_front_up(void)//前轮上升
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
}

void chsaaic_front_down(void)//前轮下降
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
}


//3区控制