//written by Fred Xiao

#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"

#define guangdian_down_1  HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13)
#define guangdian_down_2  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)
#define guangdian_up_1  HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)
#define guangdian_up_2  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)

#define LIFT_TOP_POS    490  // 上升最高位置
#define LIFT_BOT_POS    10   // 下降最低位置


typedef struct
{
    float angle;         
    float distance;        
    uint8_t btn_1;      
    uint8_t btn_2;     
    uint8_t btn_3;     
    uint8_t btn_4;     
    uint8_t update_flag;
    //遥控器

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
    //陀螺仪

    int laser_adc[4];
    //DT35

    int32_t work_mode;
    float xyz_in_base[3];
    //视觉摄像头通信



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
}chassic_control_t;

typedef struct
{
    float angle;
    float speed;

    float span;
    float lift;


    float angle_balance;
    float angle_balance_target;  // angle_balance 的目标值，用于逐度逼近
    float error_balance;


    int x, y, z;
    int angle1, angle2, angle3, angle4;

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
    int meilin_count_flag;
    int horizontal_s_flag;
    int vertical_s_flag;
    int complete_flag;
    int complete_taijie_flag;
    int complete_dingwei_flag;
    int complete_erqu_flag;
    int start_sanqugoon_flag;
    int bool_meilin_flag;

    int KFS_Get_flag;
    int KFS_status_flag;
    int KFS_Grap_flag;

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


void chassic_control_auto(chassic_control_t *chassic_data, float now_x, float now_y, float target_x, float target_y);

void zhuazi_mode_1_5(void);
void zhuazi_mode_2(void);
void zhuazi_mode_3(void);
void zhuazi_mode_4(void);
void zhuazi_mode_6(void);
void zhuazi_mode_7(void);


void chassic_up(void);
void chassic_down(void);
void chsaaic_behind_up(void);
void chsaaic_behind_down(void);
void chsaaic_front_up(void);
void chsaaic_front_down(void);

extern const float data_table[12][2];

void check_dingwei(float current_x, float current_y, int cell_index);

extern float v[4];
void v_update(void);
void check_dingwei_2(float current_x, float current_y, float target_x, float target_y);


#endif // __CONTROL_H