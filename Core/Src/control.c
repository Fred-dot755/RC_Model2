//written by Fred Xiao

#include "control.h"
#include <stdio.h>

My_extern R2_Extern;

int lift_flag = 0;
int motor1_init = 0;
int motor2_init = 0;
int chassic_mood;

void lift_control(void)
{
    if(lift_flag == 0)
    {
        if((guangdian_down_1 == 1))
        {
            motor1_init = -3000;
        }
        else
        {
            motor1_init = 0;
        }
        if((guangdian_down_2 == 1))
        {
            motor2_init = -3000;
        }
        else
        {
            motor2_init = 0;
        }

        if(guangdian_down_1 == 0 && guangdian_down_2 == 0)
        {
            lift_flag = 1;
        }
    }
    else
    {
        if(chassic_mood == 1)
        {
            if(guangdian_up_1 != 0)
            {
                motor1_init = 3000;
            }
            else
            {
                motor1_init = 0;
            }
             if(guangdian_up_2 != 0)
            {
                motor2_init = 3000;
            }
            else
            {
                motor2_init = 0;
            }

        }
        if(chassic_mood == 2)
        {
            if(guangdian_down_1 != 0)
            {
                motor1_init = -3000;
            }
            else
            {
                motor1_init = 0;
            }

            if(guangdian_down_2 != 0)
            {
                motor2_init = -3000;
            }
            else
            {
                motor2_init = 0;
            }
        }

    }
}

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