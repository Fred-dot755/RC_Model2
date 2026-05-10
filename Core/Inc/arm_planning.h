// written by Fred Xiao
#ifndef ARM_PLANNING_H
#define ARM_PLANNING_H
#include "main.h"

typedef struct {
    int angle1;
    int angle2;
    int target_angle1;
    int target_angle2;
    int err_angle1;
    int err_angle2;
    int output_angle1;
    int output_angle2;

    int grab_state;//抓取状态 0:未抓取 1:抓取
    int put_state;//放置状态 0:未放置 1:放置

    int bool_moving;//是否正在运动 0:静止 1:运动
    
    // ======== 新增 ========
    int speed_update_counter; // 速度更新频率计数器（用于高频循环下的缓启停）

    int lift;
} ArmPosition;

typedef struct {
    int mood;
} grep_mood;

extern ArmPosition now_position;
extern grep_mood now_mood;

void arm_unitree_planning_init(void);
void judgment_moving(void);
void arm_unitree_planning_update(int angle1, int angle2);
int get_smooth_speed(int target_speed, int current_speed);
void up_stair(void);

/**
 * @brief 机械臂水平移动 - 保持高度恒定，移动到目标位置
 * @param target_x  目标 x 坐标 (mm)
 * @param target_y  目标 y 坐标 (mm)，通常为 0
 * @param constant_z 保持恒定的高度 z (mm)
 */
void arm_goto_x(float target_x, float target_y, float constant_z);

/**
 * @brief 停止机械臂移动，保持当前位置
 */
void arm_goto_stop(void);

/**
 * @brief 查询 arm_goto 是否正在执行
 * @return 1=正在移动, 0=已停止
 */
int arm_goto_is_active(void);

#endif
