// written by Fred Xiao
#include "arm_planning.h"
#include "DH.h"

ArmPosition now_position;
grep_mood now_mood;

// arm_goto 使能标志：1=正在执行, 0=停止（保持当前位置）
static int arm_goto_active = 0;

// 循环越快，这个值应该越大。可以根据实际电机的顺滑程度微调（比如改成2、5、10）
#define SPEED_UPDATE_DELAY 1 

void arm_unitree_planning_init(void)
{
    now_position.angle1 = 0;
    now_position.angle3 = 0;

    now_position.target_angle1 = 0;
    now_position.target_angle3 = 0;

    now_position.output_angle1 = unitree_pos[0];
    now_position.output_angle3 = unitree_pos[1];
    
    now_position.speed_update_counter = 0;
}

int get_smooth_speed(int target_speed, int current_speed) 
{
    int diff = target_speed - current_speed;
    int max_a;
    
    if ((target_speed >= 0 && current_speed >= 0 && target_speed > current_speed) || 
        (target_speed <= 0 && current_speed <= 0 && target_speed < current_speed)) 
    {
        int abs_speed = current_speed > 0 ? current_speed : -current_speed;
        
        if (abs_speed <= 2) {
            max_a = 1;       
        } 
        else if (abs_speed <= 7) {
            max_a = 3;       
        } 
        else {
            max_a = 1;       
        }
    } 
    else 
    {
        max_a = 2; // 减速或反转时，加速度给 2
    }
    
    if (diff > max_a) return current_speed + max_a;
    if (diff < -max_a) return current_speed - max_a;
    return target_speed;
}

void judgment_moving(void)
{
    int angle_diff1 = abs(now_position.target_angle1 - now_position.output_angle1);
    int angle_diff3 = abs(now_position.target_angle3 - now_position.output_angle3);
    
    if(angle_diff1 <= 1 && angle_diff3 <= 1 &&
       now_position.err_angle1 == 0 && now_position.err_angle3 == 0)
    {
        now_position.bool_moving = 0;
    }
    else
    {
        now_position.bool_moving = 1;
    }
}

void arm_unitree_planning_update(int angle1, int angle3)
{
    now_position.target_angle1 = angle1;
    now_position.target_angle3 = angle3;

    int last_err_angle1 = now_position.err_angle1;
    int last_err_angle3 = now_position.err_angle3;

    float diff1 = now_position.target_angle1 - now_position.output_angle1;
    float diff3 = now_position.target_angle3 - now_position.output_angle3;

    while (diff1 > 180.0f) diff1 -= 360.0f;
    while (diff1 < -180.0f) diff1 += 360.0f;
    while (diff3 > 180.0f) diff3 -= 360.0f;
    while (diff3 < -180.0f) diff3 += 360.0f;

    float error1 = diff1;
    float error3 = diff3;

    int target_err1 = (int)(error1 / 20.0f);
    int target_err3 = (int)(error3 / 20.0f);

    if (target_err1 == 0 && (int)error1 != 0) target_err1 = (error1 > 0) ? 1 : -1;
    if (target_err3 == 0 && (int)error3 != 0) target_err3 = (error3 > 0) ? 1 : -1;

    if(target_err1 > 5) target_err1 = 5;
    if(target_err1 < -5) target_err1 = -5;
    if(target_err3 > 5) target_err3 = 5;
    if(target_err3 < -5) target_err3 = -5;

    now_position.speed_update_counter++;
    if (now_position.speed_update_counter >= SPEED_UPDATE_DELAY)
    {
        now_position.speed_update_counter = 0; // 重置计数器
        
        now_position.err_angle1 = get_smooth_speed(target_err1, last_err_angle1);
        now_position.err_angle3 = get_smooth_speed(target_err3, last_err_angle3);
    }
    else
    {
        now_position.err_angle1 = last_err_angle1;
        now_position.err_angle3 = last_err_angle3;
    }

    now_position.output_angle1 += now_position.err_angle1;
    now_position.output_angle3 += now_position.err_angle3;

    now_position.angle1 = unitree_pos[0];
    now_position.angle3 = unitree_pos[1];
    
    judgment_moving();
}


void up_stair(void)
{
    R2_Extern.angle2 = angle_2;
    R2_Extern.angle3 = angle_3;
    if(unitree_pos[1]>= (R2_Extern.angle3-5) && unitree_pos[1] <= (R2_Extern.angle3+5) && dm4310_fb[1].position_deg >= (R2_Extern.angle2-5) && dm4310_fb[1].position_deg <= (R2_Extern.angle2+5))
    {
        return;
    }
}




/**
 * @brief 机械臂水平移动 - 保持高度恒定，移动到目标位置
 * @param target_x  目标 x 坐标 (mm)
 * @param target_y  目标 y 坐标 (mm)，通常为 0
 * @param constant_z 保持恒定的高度 z (mm)
 *
 * 调用 inverseKinematics 计算关节角度，写入 R2_Extern.angle1/2/3，
 * 由 freertos 中 inverseKinematics_Function 线程周期执行 arm_unitree_planning_update() 完成运动。
 */
void arm_goto_x(float target_x, float target_y, float constant_z)
{
    int angle1, angle2, angle3;

    bool reachable = inverseKinematics((int)target_x, (int)target_y, (int)constant_z,
                                       &angle1, &angle2, &angle3);

    // 写入全局目标关节角，由 inverseKinematics_Function 线程执行
    R2_Extern.angle1 = angle1;
    R2_Extern.angle2 = angle2;
    R2_Extern.angle3 = angle3;

    arm_goto_active = 1;

    // 如果不可达，可在此处添加错误处理（如 LED 报警）
    (void)reachable;
}

void arm_goto_stop(void)
{
    // 停止时保持当前位置：将当前实际关节角写回目标
    R2_Extern.angle1 = (int)unitree_pos[0];
    R2_Extern.angle2 = (int)unitree_pos[1];
    // angle3 由 DM 电机反馈获取，保持当前值即可
    arm_goto_active = 0;
}

int arm_goto_is_active(void)
{
    return arm_goto_active;
}

void arm_put_kfs(void)
{
    R2_Extern.angle2 = angle_2;
    R2_Extern.angle3 = angle_3 + 20;
    if(unitree_pos[1]>= R2_Extern.angle2 - 5 && unitree_pos[1] <= R2_Extern.angle2 + 5 && dm4310_fb[1].position_deg >= R2_Extern.angle3 - 5 && dm4310_fb[1].position_deg <= R2_Extern.angle3 + 5)
      {
        R2_Extern.angle1 = 155;
        if(unitree_pos[0] >= R2_Extern.angle1 - 5 && R2_Extern.angle1 <= unitree_pos[0] + 5)
        {
            R2_Extern.angle4 = 90;
            R2_Extern.angle2 = angle_2 + 20;
        }
      }
}