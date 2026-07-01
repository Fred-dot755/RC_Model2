//written by Fred Xiao

#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "speed_plan.h"

#define guangdian_down_1  HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13)
#define guangdian_down_2  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)
#define guangdian_up_1  HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)
#define guangdian_up_2  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)

#define LIFT_TOP_POS    490  // lift top position
#define LIFT_BOT_POS    10   // lift bottom position
#define guangdian HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13)

#define Max_KFS 2

typedef struct
{
    float angle;         
    float distance;        
    uint8_t btn_1;      
    uint8_t btn_2;     
    uint8_t btn_3;     
    uint8_t btn_4;     
    uint8_t update_flag;
    // remote controller
    float HIPNUCAngleX;
	float HIPNUCAngleY;
	float HIPNUCAngleZ;
	float HIPNUCQuaternionsW;
	float HIPNUCQuaternionsX;
	float HIPNUCQuaternionsY;
	float HIPNUCQuaternionsZ;
	float HIPNUCgyroscopeX;
	float HIPNUCgyroscopeY;
	float HIPNUCgyroscopeZ;
	float HIPNUCaccelerationX;
	float HIPNUCaccelerationY;
	float HIPNUCaccelerationZ;
    // IMU
    int laser_adc[4];
    //DT35

    int32_t work_mode;
    float xyz_in_base[3];
    // vision camera communication


}monitor_data;

extern int lift_flag;
extern int motor1_init;
extern int motor2_init;
extern int chassic_mood;



void lift_control(void);

typedef enum {
    STATE_ONE,
    STATE_TWO,
    STATE_THREE,
    STATE_FOUR,
    STATE_FIVE,
    STATE_SIX
} StairState_t;

extern StairState_t current_state;

typedef struct 
{
    float now_x;
    float now_y;
    float target_x;
    float target_y;

    float angle;
    float distance;
    speed_plan_t speed_plan;
}chassic_control_t;

typedef struct
{
    float angle;
    float speed;

    float span;
    float lift;


    float angle_balance;
    float angle_balance_target;  // target of angle_balance
    float error_balance;


    int x, y, z;
    int angle1, angle2, angle3, angle4;
    int angle5;

    int pitch_angle;
    int yaw_angle;

    int x_t, y_t, z_t;

    int lift_mood;

    int get_init;

    int lift_flag;

    int Area2_flag;

    int chsaaic_behind_flag;
    int chsaaic_front_flag;

    int check_1_flag;
    int bool_check_1_flag;

    int chack_yaw_flag;

    int check_angle1_flag;
    int check_angle2_flag;
    int check_angle3_flag;
    int check_angle4_flag;
    int check_angle5_flag;

    int work_mode;

    int Track_Mode;
    int Track_flag;

    //area1
    int Area1_flag;
    int Area1_step;

    int Area1_1_flag; // start waypoint flag
    int Area1_2_flag; // laser align flag
    int Area1_timer;
    int Area1_id; // waypoint id
    float Area1_dx; // offset

    int Area1_qukuang_flag;



    //area2
    int meilin_count_flag;
    int horizontal_s_flag;
    int vertical_s_flag;
    int complete_flag;
    int complete_taijie_flag;
    int complete_dingwei_flag;
    int complete_erqu_flag;
    int start_sanqugoon_flag;
    int bool_meilin_flag;
    int sanqugoon_step;

    int KFS_Get_flag;
    int KFS_status_flag;
    int KFS_Grap_flag;
    int bool_KFS_flag;
    int car_flag;
    int qukuang_mode_flag;
    int special_flag;

    int kfs_count;
    int kfs_count_total;

    //area3
    int Area3_flag;
    int Area3_step;
    int Area3_cell;
    int Area3_2_flag;

    int lingshi_flag;

}My_extern;



typedef struct 
{
    int qian_dis;
    int hou_dis;
}Area2_control_t;


extern chassic_control_t chassic_data;
extern My_extern R2_Extern;
extern Area2_control_t Area_Flag;


void chassic_control_auto(chassic_control_t *chassic_data, float now_x, float now_y, float target_x, float target_y , float max_speed);


void quzhua(float x, float y);
void quzhua_dui(float y, float x);
void fangkuang(float x, float y);
void qukuang_move_y(float y);
void back_keep_x(float x, float angle, float speed);
void back_keep_y(float y, float angle, float speed);


void chassic_up(void);
void chassic_down(void);
void chsaaic_behind_up(void);
void chsaaic_behind_down(void);
void chsaaic_front_up(void);
void chsaaic_front_down(void);
void zhuazi_open(void);
void zhuazi_close(void);
void fangkuang_open_1(void);
void fangkuang_close_1(void);
void fangkuang_open_2(void);
void fangkuang_close_2(void);
void qibeng_open(void);
void qibeng_close(void);


extern const float area_1_dt35_red[6][2];
extern const float area_1_dt35_blue[6][2];
extern const float area_3_dt35_red[3][2];
extern const float area_3_dt35_blue[3][2];
extern const float area_1_red[10][2];
extern const float area_2_red[10][2];
extern const float area_2_special_red[3][2];
extern const float area_3_red[10][2];
extern const float area_1_blue[10][2];
extern const float area_2_blue[10][2];
extern const float area_2_special_blue[3][2];
extern const float area_3_blue[10][2];
extern const float data_table_red[12][2];
extern const float data_table_blue[12][2];

void check_dingwei(float current_x, float current_y, int cell_index);

extern float v[4];
void v_update(void);
void check_dingwei_2(float current_x, float current_y, float target_x, float target_y);


void put_kfs(void);
void put_kfs_1(void);
void put_kfs_2(void);
void put_kfs_3(void);


#endif // __CONTROL_H
