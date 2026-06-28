//written by Fred Xiao

#include "control.h"
#include <stdio.h>

My_extern R2_Extern;
Area2_control_t Area_Flag;

StairState_t current_state = STATE_SIX;


chassic_control_t chassic_data;
static speed_plan_t quzhua_speed_plan;
static float quzhua_plan_target_x = 0.0f;
static float quzhua_plan_target_y = 0.0f;

void chassic_control_auto(chassic_control_t *chassic_data, float now_x, float now_y, float target_x, float target_y , float max_speed)
{
    const float DEADZONE = 0.02f;
    const float TARGET_EPSILON = 0.005f;

    chassic_data->now_x = now_x;
    chassic_data->now_y = now_y;
    chassic_data->target_x = target_x;
    chassic_data->target_y = target_y;

    float dx = target_x - now_x;
    float dy = target_y - now_y;
    float real_distance = sqrtf(dx * dx + dy * dy);

    int raw_angle = (int)(atan2f(dy, dx) * 180.0f / PI);
    if (raw_angle > 180) {
        raw_angle -= 360;
    }
    chassic_data->angle = -raw_angle;

    if (real_distance < DEADZONE || max_speed <= 0.0f)
    {
        speed_plan_reset(&chassic_data->speed_plan);
        chassic_data->distance = 0.0f;
        return;
    }

    uint32_t now_ms = HAL_GetTick();
    if (speed_plan_need_replan(&chassic_data->speed_plan,
                               target_x, target_y,
                               TARGET_EPSILON))
    {
        speed_plan_start(&chassic_data->speed_plan,
                         now_x, now_y,
                         target_x, target_y,
                         max_speed,
                         now_ms);
    }

    chassic_data->distance = speed_plan_update(&chassic_data->speed_plan,
                                               now_ms,
                                               max_speed);
}

// area1 control
// const float area_1_dt35[6][2] = {
//     {412.0f, 216.0f},
//     {610.0f, 215.0f},
//     {808.0f, 219.0f},
//     {1005.0f, 212.0f},
//     {1210.0f, 334.0f},
//     {1426.0f, 334.0f}
// };
const float area_1_dt35_red[6][2] = {
    {0.0f, 0.0f},
    {50.0f, 204.0f},
    {236.0f, 200.0f},
    {435.0f, 201.0f},
    {621.0f, 201.0f},
    {830.0f, 218.0f}
};

const float area_1_dt35_blue[6][2] = {
    {392.0f, 206.0f},
    {590.0f, 205.0f},
    {788.0f, 209.0f},
    {985.0f, 202.0f},
    {1190.0f, 324.0f},
    {1406.0f, 324.0f}
};

void quzhua(float x, float y)
{
    const float DEADZONE_MM = 1.0f;
    const float MAX_SPEED_M = 0.5f;
    const float MIN_SPEED_M = 0.12f;
    const float TARGET_EPSILON_MM = 0.5f;

    int x_dist_index = (visual_data.hmi_color == 2) ? 1 : 2;

    if (dt35_data.dist[x_dist_index] < 10 || dt35_data.dist[x_dist_index] > 5900 ||
        dt35_data.dist[3] < 10 || dt35_data.dist[3] > 5900)
    {
        return;
    }

    float x_dist_mm = (float)dt35_data.dist[x_dist_index];
    float back_dist_mm = (float)dt35_data.dist[3];

    // float left_error_mm = (visual_data.hmi_color == 2) ? (x - x_dist_mm) : (x_dist_mm - x);
    // float forward_error_mm = (visual_data.hmi_color == 2) ? (y - back_dist_mm) : (y - back_dist_mm);
    float left_error_mm = x_dist_mm - x;
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
        speed_plan_reset(&quzhua_speed_plan);
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

    R2_Extern.angle = atan2f(left_error_mm, forward_error_mm) * 180.0f / PI;

    uint32_t now_ms = HAL_GetTick();
    if (quzhua_speed_plan.active == 0U ||
        fabsf(quzhua_plan_target_x - x) > TARGET_EPSILON_MM ||
        fabsf(quzhua_plan_target_y - y) > TARGET_EPSILON_MM)
    {
        quzhua_plan_target_x = x;
        quzhua_plan_target_y = y;
        speed_plan_start(&quzhua_speed_plan,
                         0.0f, 0.0f,
                         distance_m, 0.0f,
                         MAX_SPEED_M,
                         now_ms);
    }

    R2_Extern.speed = speed_plan_update(&quzhua_speed_plan,
                                        now_ms,
                                        MAX_SPEED_M);

    if (R2_Extern.speed > 0.0f && R2_Extern.speed < MIN_SPEED_M)
    {
        R2_Extern.speed = MIN_SPEED_M;
    }
}

void quzhua_dui(float y, float x)
{
    const float DEADZONE_MM = 2.0f;
    const float MAX_SPEED_M = 0.5f;
    const float SLOW_DIST_M = 0.5f;

    if (dt35_data.dist[3] < 10 || dt35_data.dist[3] > 5900)
    {
        return;
    }

    float back_dist_mm = (float)dt35_data.dist[3];
    float left_error_mm = x;
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

    R2_Extern.angle = atan2f(left_error_mm, forward_error_mm) * 180.0f / PI;

    if (distance_m >= SLOW_DIST_M)
        R2_Extern.speed = MAX_SPEED_M;
    else
        R2_Extern.speed = MAX_SPEED_M * (distance_m / SLOW_DIST_M) * 2.5f;

    if(R2_Extern.speed < 0.1f)
    {
        R2_Extern.speed = 0.1f;
    }
}


void back_keep_x(float x, float angle, float speed)
{
    const float DEADZONE_MM = 2.0f;
    const float MAX_X_SPEED_M = 0.5f;
    const float SLOW_DIST_M = 0.5f;

    int x_dist_index = (visual_data.hmi_color == 2) ? 1 : 2;
    float x_dist_mm = (float)dt35_data.dist[x_dist_index];
    float left_error_mm = (visual_data.hmi_color == 2) ? (x - x_dist_mm) : (x_dist_mm - x);
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
// area2 control
void chassic_up(void) // lift up
{
    R2_Extern.lift_mood = 1;
}

void chassic_down(void) // lift down
{
    R2_Extern.lift_mood = 0;
}

//PD10 PB11
void chsaaic_behind_up(void) // rear wheel down
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
}
void chsaaic_behind_down(void) // rear wheel up
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
}
void chsaaic_front_up(void) // front wheel down
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
}
void chsaaic_front_down(void) // front wheel up
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
}
//PB10 PD7 PB4 PB3 PE15
void zhuazi_open(void) // claw open
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
}
void zhuazi_close(void) // claw close
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
}
void fangkuang_open(void) // mine release open
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
}
void fangkuang_close(void) // close mine release
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
}

// red area
const float area_1_red[10][2] = {
    {0.62f,0.39f}
};

const float area_2_red[10][2] = {
    {8.32f,-4.0f},
    {11.02f,-4.24f}
};

const float area_3_red[10][2] = {
    {8.32f,-4.0f}
};

// blue area
const float area_1_blue[10][2] = {
    {0.62f,-0.39f}
};

const float area_2_blue[10][2] = {
    {7.89f,4.89f},
    {10.60f,5.54f}
};


const float area_3_blue[10][2] = {
    {8.32f,4.0f}
};


const float data_table_red[12][2] = {
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

const float data_table_blue[12][2] = {
    {3.48f, 0.72f},//1
    {3.29f, 1.90f},//2
    {3.14f, 3.09f},//3   
    {4.34f, 3.32f},//6
    {4.44f, 2.14f},//5
    {4.67f, 0.90f},//4
    {5.51f, 3.49f},//9
    {5.69f, 2.27f},//8
    {5.81f, 1.12f},//7
    {6.65f, 3.67f},//12
    {6.84f, 2.53f},//11
    {7.06f, 1.34f}//10
};


void check_dingwei(float current_x, float current_y, int cell_index)
{
    // cell_index: 0~11, maps to data_table[0]~data_table[11]
    if (cell_index < 0 || cell_index >= 12)
    {
        R2_Extern.complete_dingwei_flag = 0;
        return;
    }

    const float (*table)[2] = (R2_Extern.Track_flag == 0) ? data_table_red : data_table_blue;

    float dx = current_x - table[cell_index][0];
    float dy = current_y - table[cell_index][1];
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

// area3 control
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
