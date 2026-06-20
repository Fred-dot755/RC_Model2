//written by Fred Xiao

#include "control.h"
#include <stdio.h>

My_extern R2_Extern;
Area2_control_t Area_Flag;

StairState_t current_state = STATE_SIX;


chassic_control_t chassic_data;

void chassic_control_auto(chassic_control_t *chassic_data, float now_x, float now_y, float target_x, float target_y , float max_speed)
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
    
    float MAX_SPEED = max_speed; 
    double ONE_METER = 0.5;
    double DEADZONE = 0.02;

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
        chassic_data->distance = 2 * real_distance;
    }
}



//1区控制
const float area_1_dt35[6][2] = {
    {412.0f, 212.0f},
    {620.0f, 212.0f},
    {812.0f, 212.0f},
    {1044.0f, 212.0f},
    {1218.0f, 212.0f},
    {1426.0f, 389.0f}
};

void quzhua(float x, float y)
{
    const float DEADZONE_MM = 2.0f;
    const float MAX_SPEED_M = 0.5f;
    const float SLOW_DIST_M = 0.5f;

    if (dt35_data.dist[2] < 10 || dt35_data.dist[2] > 5900 ||
        dt35_data.dist[3] < 10 || dt35_data.dist[3] > 5900)
    {
        return;
    }

    float right_dist_mm = (float)dt35_data.dist[2];
    float back_dist_mm = (float)dt35_data.dist[3];

    float left_error_mm = x - right_dist_mm;
    float forward_error_mm = y - back_dist_mm;

    if (fabsf(left_error_mm) <= DEADZONE_MM)
    {
        left_error_mm = 0.0f;
    }

    if (fabsf(forward_error_mm) <= DEADZONE_MM)
    {
        forward_error_mm = 0.0f;
    }

    if (left_error_mm == 0.0f && forward_error_mm == 0.0f)
    {
        R2_Extern.angle = 0.0f;
        R2_Extern.speed = 0.0f;
        R2_Extern.Area1_2_flag = 1;
        return;
    }
    else
    {
        R2_Extern.Area1_2_flag = 0;
    }

    float distance_mm = sqrtf(left_error_mm * left_error_mm + forward_error_mm * forward_error_mm);
    float distance_m = distance_mm / 1000.0f;

    R2_Extern.angle = atan2f(-left_error_mm, forward_error_mm) * 180.0f / PI;

    if (distance_m >= SLOW_DIST_M)
        R2_Extern.speed = MAX_SPEED_M;
    else
        R2_Extern.speed = MAX_SPEED_M * (distance_m / SLOW_DIST_M) * 2.5;
        if(R2_Extern.speed < 0.1)
        {
            R2_Extern.speed = 0.1;
        }
}


void back_keep_x(float x, float angle, float speed)
{
    const float DEADZONE_MM = 2.0f;
    const float MAX_X_SPEED_M = 0.5f;
    const float SLOW_DIST_M = 0.5f;

    float right_dist_mm = (float)dt35_data.dist[2];
    float left_error_mm = x - right_dist_mm;
    float left_adjust_speed = 0.0f;

    if (fabsf(left_error_mm) <= DEADZONE_MM)
    {
        R2_Extern.Area1_2_flag = 1;
    }
    else
    {
        R2_Extern.Area1_2_flag = 0;

        float distance_m = fabsf(left_error_mm) / 1000.0f;
        if (distance_m >= SLOW_DIST_M)
            left_adjust_speed = MAX_X_SPEED_M;
        else
            left_adjust_speed = MAX_X_SPEED_M * (distance_m / SLOW_DIST_M) * 2.5f;

        if (left_adjust_speed > MAX_X_SPEED_M)
            left_adjust_speed = MAX_X_SPEED_M;

        if (left_error_mm < 0.0f)
            left_adjust_speed = -left_adjust_speed;
    }

    float angle_rad = angle * PI / 180.0f;
    float forward_speed = speed * cosf(angle_rad);
    float left_speed = -speed * sinf(angle_rad) + left_adjust_speed;

    R2_Extern.angle = atan2f(-left_speed, forward_speed) * 180.0f / PI;
    R2_Extern.speed = sqrtf(forward_speed * forward_speed + left_speed * left_speed);
    if(R2_Extern.speed < 0.2)
        {
            R2_Extern.speed = 0.2;
        }
}


void back_keep_y(float y, float angle, float speed)
{
    const float DEADZONE_MM = 1.0f;
    const float MAX_Y_SPEED_M = 0.3f;
    const float SLOW_DIST_M = 0.1f;

    if (dt35_data.dist[3] < 10 || dt35_data.dist[3] > 5900)
    {
        return;
    }

    float back_dist_mm = (float)dt35_data.dist[3];
    float forward_error_mm = y - back_dist_mm;
    float forward_adjust_speed = 0.0f;

    if (fabsf(forward_error_mm) <= DEADZONE_MM)
    {
        R2_Extern.Area1_2_flag = 1;
    }
    else
    {
        R2_Extern.Area1_2_flag = 0;

        float distance_m = fabsf(forward_error_mm) / 1000.0f;
        if (distance_m >= SLOW_DIST_M)
            forward_adjust_speed = MAX_Y_SPEED_M;
        else
            forward_adjust_speed = MAX_Y_SPEED_M * (distance_m / SLOW_DIST_M) * 1.5f;

        if (forward_adjust_speed > MAX_Y_SPEED_M)
            forward_adjust_speed = MAX_Y_SPEED_M;

        if (forward_error_mm < 0.0f)
            forward_adjust_speed = -forward_adjust_speed;
    }

    float angle_rad = angle * PI / 180.0f;
    float forward_speed = speed * cosf(angle_rad) + forward_adjust_speed;
    float left_speed = -speed * sinf(angle_rad);

    R2_Extern.angle = atan2f(-left_speed, forward_speed) * 180.0f / PI;
    R2_Extern.speed = sqrtf(forward_speed * forward_speed + left_speed * left_speed);
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

//PD10 PB11
void chsaaic_behind_up(void)//后轮下降
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
}
void chsaaic_behind_down(void)//后轮上升
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
}
void chsaaic_front_up(void)//前轮下降
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
}
void chsaaic_front_down(void)//前轮上升
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
}
//PB10 PD7 PB4 PB3 PE15
void zhuazi_open(void)//爪子打开
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
}
void zhuazi_close(void)//爪子关闭
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
}
void fangkuang_open(void)//放矿
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
}
void fangkuang_close(void)//放矿关闭
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
}

const float area_1[10][2] = {
    {0.62f,0.38f}
};

const float area_2[10][2] = {
    {8.32f,-4.0f},
    {11.02f,-4.24f}
};

const float area_3[10][2] = {
    {8.32f,-4.0f}
};

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
const float area_3_dt35[6][2] = {
    {460.0,0},
    {962.0,0},
    {1570.0,0}
};

void put_kfs(void)
{
    R2_Extern.angle2 = 12;
    R2_Extern.angle3 = 75;
    R2_Extern.angle4 = -25;
}

void put_kfs_1(void)
{
    R2_Extern.angle2 = 0;
    R2_Extern.angle3 = 110;
    R2_Extern.angle4 = -90;
}

void put_kfs_2(void)
{
    R2_Extern.angle2 = 5;
    R2_Extern.angle3 = 110;
    R2_Extern.angle4 = -15;
}

void put_kfs_3(void)
{
    arm_goto_x(100,0,810);
}

float v[4];
void v_update(void)
{
    v[0] = -(float)motor[0].NowSpeed * CUR_TO_RPM;
    v[1] = -(float)motor[1].NowSpeed * CUR_TO_RPM;
    v[2] = -(float)motor[2].NowSpeed * CUR_TO_RPM;
    v[3] = -(float)motor[3].NowSpeed * CUR_TO_RPM;
}
