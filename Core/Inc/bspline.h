/**
 * @file    bspline.h
 * @brief   B样条插值算法 - 用于底盘路径规划与平滑运动控制
 * @details 提供三次均匀B样条（Cubic Uniform B-Spline）的构建、求值与路径跟踪功能。
 *          配合 chassic_control_auto() 使用，使底盘沿平滑曲线运动而非直线。
 * 
 * @note    适用于 STM32H7 嵌入式平台，使用 float 单精度运算，无需动态内存分配。
 *          （在"freertos.c"的 Mid360_Function 中，原本调用：
 *           chassic_control_auto(&chassic_data, visual_data.x_map, visual_data.y_map,
 *                                chassic_data.target_x, chassic_data.target_y);
 *           使用本模块后，可通过 B样条路径规划生成中间插值点，逐点送入上述函数。）
 */

#ifndef __BSPLINE_H
#define __BSPLINE_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== 配置宏 ========================== */

/** @brief B样条曲线度数（3 = 三次B样条，推荐值） */
#define BSPLINE_DEGREE         3

/** @brief 最大控制点数量（决定路径形状的顶点数） */
#define BSPLINE_MAX_CTRL_PTS   20

/** @brief 路径预采样点数（控制点之间插值的精细程度） */
#define BSPLINE_PATH_SAMPLES   100

/** @brief 路径跟踪完成阈值（距离目标点小于此值视为到达） */
#define BSPLINE_ARRIVE_THRESH  0.02f

/* ======================= 数据结构 ======================= */

/**
 * @brief B样条曲线对象
 * @note  存储控制点与节点向量，提供曲线求值接口
 */
typedef struct {
    float ctrl_x[BSPLINE_MAX_CTRL_PTS]; /**< 控制点 X 坐标 */
    float ctrl_y[BSPLINE_MAX_CTRL_PTS]; /**< 控制点 Y 坐标 */
    int   num_ctrl;                     /**< 实际控制点数量 */
    int   degree;                       /**< B样条度数（默认 3） */
    float knots[BSPLINE_MAX_CTRL_PTS + BSPLINE_DEGREE + 1]; /**< 节点向量 */
    int   num_knots;                    /**< 节点向量长度 */
    int   initialized;                  /**< 初始化标志 */
} BSpline2D;

/**
 * @brief 预采样路径点（用于快速跟踪，避免实时求值）
 */
typedef struct {
    float x[BSPLINE_PATH_SAMPLES]; /**< 采样点 X 坐标 */
    float y[BSPLINE_PATH_SAMPLES]; /**< 采样点 Y 坐标 */
    int   num_samples;             /**< 实际采样点数 */
} BSPath;

/**
 * @brief 路径跟踪器状态
 * @note  沿预采样路径依次前进，每次调用返回下一个目标点
 */
typedef struct {
    BSPath    path;              /**< 预采样路径 */
    int       current_index;     /**< 当前目标点索引 */
    float     start_x;           /**< 起始位置 X (用于判断是否需要重新规划) */
    float     start_y;           /**< 起始位置 Y */
    float     target_x;          /**< 最终目标 X */
    float     target_y;          /**< 最终目标 Y */
    uint8_t   active;            /**< 跟踪器是否激活 */
    uint8_t   arrived;           /**< 是否到达终点 */
} BSTracker;

/* ===================== 函数声明 ===================== */

/**
 * @brief 初始化 B样条曲线（Clamped Cubic B-Spline）
 * @param bs       B样条对象指针
 * @param ctrl_x   控制点 X 坐标数组
 * @param ctrl_y   控制点 Y 坐标数组
 * @param num      控制点数量（需 >= 4）
 * @param degree   曲线度数（推荐 3）
 * @retval 0 成功，-1 失败（参数错误）
 */
int bspline_init(BSpline2D *bs, const float *ctrl_x, const float *ctrl_y,
                 int num, int degree);

/**
 * @brief 根据参数 t 求取 B样条曲线上的点
 * @param bs   B样条对象指针
 * @param t    参数值 [0, 1] (0 对应第一个控制点, 1 对应最后一个)
 * @param x    输出 X 坐标
 * @param y    输出 Y 坐标
 * @retval 0 成功，-1 失败
 */
int bspline_eval(const BSpline2D *bs, float t, float *x, float *y);

/**
 * @brief 将 B样条曲线预采样为离散路径点
 * @param bs    B样条对象指针
 * @param path  输出路径点数组
 * @param n     采样点数
 * @retval 0 成功，-1 失败
 */
int bspline_sample_path(const BSpline2D *bs, BSPath *path, int n);

/**
 * @brief 快速创建从起点到终点的平滑路径（通过插入中间控制点实现曲线）
 * @param start_x     起点 X
 * @param start_y     起点 Y
 * @param target_x    终点 X
 * @param target_y    终点 Y
 * @param mid_x       中间控制点 X（可为 NULL，传 NULL 时将自动生成直线辅助点）
 * @param mid_y       中间控制点 Y（可为 NULL）
 * @param num_mid     中间控制点数量（为 0 时自动生成 2 个辅助点使路径平滑）
 * @param path        输出路径（预采样点）
 * @param num_samples 采样点数（推荐 BSPLINE_PATH_SAMPLES）
 * @retval 0 成功，-1 失败
 */
int bspline_plan_path(float start_x, float start_y,
                      float target_x, float target_y,
                      const float *mid_x, const float *mid_y, int num_mid,
                      BSPath *path, int num_samples);

/**
 * @brief 初始化路径跟踪器
 * @param tracker  跟踪器对象指针
 * @param path     预采样路径
 * @param start_x  起始位置 X
 * @param start_y  起始位置 Y
 * @param target_x 最终目标 X
 * @param target_y 最终目标 Y
 */
void bspline_tracker_init(BSTracker *tracker, const BSPath *path,
                          float start_x, float start_y,
                          float target_x, float target_y);

/**
 * @brief 获取跟踪器的下一个目标点
 * @param tracker 跟踪器对象指针
 * @param now_x   当前位置 X（用于判断是否到达当前目标点）
 * @param now_y   当前位置 Y
 * @param out_x   输出下一个目标点 X
 * @param out_y   输出下一个目标点 Y
 * @retval 1 正在跟踪中（输出有效）
 * @retval 0 已到达终点（输出为最终目标点）
 * @retval -1 跟踪器未激活
 */
int bspline_tracker_next(BSTracker *tracker,
                         float now_x, float now_y,
                         float *out_x, float *out_y);

/**
 * @brief 重置跟踪器（可重新设置新的起点和路径）
 * @param tracker 跟踪器对象指针
 */
void bspline_tracker_reset(BSTracker *tracker);

/**
 * @brief 检查跟踪器是否已完成任务
 * @param tracker 跟踪器对象指针
 * @retval 1 已到达终点
 * @retval 0 仍在跟踪中
 */
uint8_t bspline_tracker_arrived(const BSTracker *tracker);

#ifdef __cplusplus
}
#endif

#endif /* __BSPLINE_H */
