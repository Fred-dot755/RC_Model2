/**
 * @file    bspline.c
 * @brief   B样条插值算法实现 - 用于底盘路径规划与平滑运动控制
 * @details 实现三次均匀B样条（Cubic Uniform B-Spline）的构建、求值与路径跟踪。
 *          核心算法使用 Cox-de Boor 递归公式计算基函数。
 * 
 * @note    配合 chassic_control_auto() 使用示例：
 * 
 *          // 1. 规划路径（从当前位置到目标位置）
 *          BSPath path;
 *          bspline_plan_path(now_x, now_y, target_x, target_y,
 *                            NULL, NULL, 0, &path, BSPLINE_PATH_SAMPLES);
 * 
 *          // 2. 初始化跟踪器
 *          BSTracker tracker;
 *          bspline_tracker_init(&tracker, &path, now_x, now_y, target_x, target_y);
 * 
 *          // 3. 在控制循环中逐点跟踪
 *          float next_x, next_y;
 *          int ret = bspline_tracker_next(&tracker,
 *                                         visual_data.x_map, visual_data.y_map,
 *                                         &next_x, &next_y);
 *          if (ret == 1) {
 *              chassic_control_auto(&chassic_data,
 *                                   visual_data.x_map, visual_data.y_map,
 *                                   next_x, next_y);
 *          }
 */

#include "bspline.h"

/* ======================= 内部辅助函数 ======================= */

/**
 * @brief 计算 B样条基函数 N_{i,p}(u) 的 Cox-de Boor 递归
 * @param knots  节点向量
 * @param i      基函数索引
 * @param p      度数
 * @param u      参数值
 * @return       基函数值 N_{i,p}(u)
 */
static float de_boor_basis(const float *knots, int i, int p, float u)
{
    if (p == 0)
    {
        /* 0 次基函数：在节点区间内为 1，否则为 0 */
        return (u >= knots[i] && u < knots[i + 1]) ? 1.0f : 0.0f;
    }

    float left  = 0.0f;
    float right = 0.0f;

    /* 第一项：(u - u_i) / (u_{i+p} - u_i) * N_{i,p-1}(u) */
    float denom_left = knots[i + p] - knots[i];
    if (denom_left > 1e-10f)
    {
        left = (u - knots[i]) / denom_left * de_boor_basis(knots, i, p - 1, u);
    }

    /* 第二项：(u_{i+p+1} - u) / (u_{i+p+1} - u_{i+1}) * N_{i+1,p-1}(u) */
    float denom_right = knots[i + p + 1] - knots[i + 1];
    if (denom_right > 1e-10f)
    {
        right = (knots[i + p + 1] - u) / denom_right * de_boor_basis(knots, i + 1, p - 1, u);
    }

    return left + right;
}

/**
 * @brief 构建 clamped 节点向量（均匀内部节点）
 * @param knots  输出节点向量
 * @param n      控制点数量
 * @param p      度数
 */
static void build_clamped_knots(float *knots, int n, int p)
{
    int m = n + p + 1;  /* 节点向量长度 */

    /* 前 p+1 个节点为 0（clamped 起始） */
    for (int i = 0; i <= p; i++)
    {
        knots[i] = 0.0f;
    }

    /* 内部节点均匀分布 */
    int interior_count = n - p;  /* 内部节点段数 */
    for (int i = 0; i < interior_count - 1; i++)
    {
        knots[p + 1 + i] = (float)(i + 1);
    }

    /* 后 p+1 个节点为最大值（clamped 结束） */
    float max_knot = (float)(interior_count - 1);
    if (max_knot < 0.0f) max_knot = 0.0f;
    for (int i = 0; i <= p; i++)
    {
        knots[m - 1 - i] = max_knot;
    }
}

/**
 * @brief 在参数值 u 处求取 B样条曲线上的点
 * @param bs    B样条对象指针
 * @param u     参数值（在节点向量范围内）
 * @param x     输出 X 坐标
 * @param y     输出 Y 坐标
 * @retval 0 成功，-1 失败
 */
static int bspline_eval_u(const BSpline2D *bs, float u, float *x, float *y)
{
    if (!bs || !bs->initialized) return -1;

    float sum_x = 0.0f;
    float sum_y = 0.0f;

    for (int i = 0; i < bs->num_ctrl; i++)
    {
        float basis = de_boor_basis(bs->knots, i, bs->degree, u);
        sum_x += basis * bs->ctrl_x[i];
        sum_y += basis * bs->ctrl_y[i];
    }

    *x = sum_x;
    *y = sum_y;
    return 0;
}

/* ======================= 公共 API 实现 ======================= */

int bspline_init(BSpline2D *bs, const float *ctrl_x, const float *ctrl_y,
                 int num, int degree)
{
    if (!bs || !ctrl_x || !ctrl_y) return -1;
    if (num < degree + 1 || num > BSPLINE_MAX_CTRL_PTS) return -1;
    if (degree < 1 || degree > BSPLINE_DEGREE) return -1;

    bs->num_ctrl  = num;
    bs->degree    = degree;
    bs->initialized = 1;

    /* 复制控制点 */
    for (int i = 0; i < num; i++)
    {
        bs->ctrl_x[i] = ctrl_x[i];
        bs->ctrl_y[i] = ctrl_y[i];
    }

    /* 构建 clamped 节点向量 */
    bs->num_knots = num + degree + 1;
    build_clamped_knots(bs->knots, num, degree);

    return 0;
}

int bspline_eval(const BSpline2D *bs, float t, float *x, float *y)
{
    if (!bs || !bs->initialized || !x || !y) return -1;

    /* 将用户参数 t ∈ [0, 1] 映射到实际节点范围 */
    float t_min = bs->knots[bs->degree];
    float t_max = bs->knots[bs->num_ctrl];  /* knots[num_ctrl] 是最后一个 clamped 起始点 */
    float u = t_min + t * (t_max - t_min);

    /* 边界处理 */
    if (u < bs->knots[0])                u = bs->knots[0];
    if (u > bs->knots[bs->num_knots - 1]) u = bs->knots[bs->num_knots - 1];

    return bspline_eval_u(bs, u, x, y);
}

int bspline_sample_path(const BSpline2D *bs, BSPath *path, int n)
{
    if (!bs || !bs->initialized || !path) return -1;
    if (n < 2 || n > BSPLINE_PATH_SAMPLES) return -1;

    path->num_samples = n;

    for (int i = 0; i < n; i++)
    {
        float t = (float)i / (float)(n - 1);
        bspline_eval(bs, t, &path->x[i], &path->y[i]);
    }

    return 0;
}

int bspline_plan_path(float start_x, float start_y,
                      float target_x, float target_y,
                      const float *mid_x, const float *mid_y, int num_mid,
                      BSPath *path, int num_samples)
{
    if (!path) return -1;
    if (num_samples < 2 || num_samples > BSPLINE_PATH_SAMPLES) return -1;

    /* 确定控制点数量 */
    int num_ctrl;
    float ctrl_x[BSPLINE_MAX_CTRL_PTS];
    float ctrl_y[BSPLINE_MAX_CTRL_PTS];

    /*
     * 构建扩展控制点序列:
     * 三次 B样条至少需要 4 个控制点。
     * 当没有提供中间控制点时，自动生成 2 个辅助点，
     * 使路径在起点和终点之间形成自然的 S 形平滑过渡。
     *
     * 辅助点的生成策略:
     * 1. 计算起点到终点的方向向量
     * 2. 在垂直方向上偏移，形成平滑弧线
     * 3. 偏移量基于距离的 1/3，保证曲线平滑不过度
     */
    if (num_mid <= 0 || mid_x == NULL || mid_y == NULL)
    {
        /* 自动生成 2 个辅助控制点 */
        num_ctrl = 4;

        float dx = target_x - start_x;
        float dy = target_y - start_y;
        float dist = sqrtf(dx * dx + dy * dy);

        /* 第一个控制点 = 起点 */
        ctrl_x[0] = start_x;
        ctrl_y[0] = start_y;

        if (dist < 1e-6f)
        {
            /* 起点终点重合，直接使用直线 */
            ctrl_x[1] = start_x;
            ctrl_y[1] = start_y;
            ctrl_x[2] = target_x;
            ctrl_y[2] = target_y;
        }
        else
        {
            /* 归一化方向向量 */
            float nx = dx / dist;
            float ny = dy / dist;

            /* 垂直方向（左法向量） */
            float px = -ny;
            float py =  nx;

            /* 
             * 两个中间控制点：
             * 在路径 1/3 和 2/3 处，沿法向量偏移 dist/4
             * 形成平滑弧线
             */
            float offset = dist * 0.25f;
            if (offset < 0.1f) offset = 0.1f;

            ctrl_x[1] = start_x  + dx * 0.33f + px * offset;
            ctrl_y[1] = start_y  + dy * 0.33f + py * offset;

            ctrl_x[2] = start_x  + dx * 0.67f + px * offset;
            ctrl_y[2] = start_y  + dy * 0.67f + py * offset;
        }

        /* 最后一个控制点 = 目标点 */
        ctrl_x[3] = target_x;
        ctrl_y[3] = target_y;
    }
    else
    {
        /*
         * 用户提供了中间控制点。
         * 构建序列：起点 + 中间控制点 + 终点
         */
        num_ctrl = 2 + num_mid;
        if (num_ctrl > BSPLINE_MAX_CTRL_PTS)
        {
            num_ctrl = BSPLINE_MAX_CTRL_PTS;
            num_mid  = num_ctrl - 2;
        }

        ctrl_x[0] = start_x;
        ctrl_y[0] = start_y;

        for (int i = 0; i < num_mid; i++)
        {
            ctrl_x[1 + i] = mid_x[i];
            ctrl_y[1 + i] = mid_y[i];
        }

        ctrl_x[num_ctrl - 1] = target_x;
        ctrl_y[num_ctrl - 1] = target_y;
    }

    /* 初始化 B样条 */
    BSpline2D bs;
    if (bspline_init(&bs, ctrl_x, ctrl_y, num_ctrl, BSPLINE_DEGREE) != 0)
    {
        return -1;
    }

    /* 采样生成路径点 */
    return bspline_sample_path(&bs, path, num_samples);
}

void bspline_tracker_init(BSTracker *tracker, const BSPath *path,
                          float start_x, float start_y,
                          float target_x, float target_y)
{
    if (!tracker || !path) return;

    /* 复制路径 */
    tracker->path.num_samples = path->num_samples;
    for (int i = 0; i < path->num_samples; i++)
    {
        tracker->path.x[i] = path->x[i];
        tracker->path.y[i] = path->y[i];
    }

    tracker->current_index  = 0;
    tracker->start_x        = start_x;
    tracker->start_y        = start_y;
    tracker->target_x       = target_x;
    tracker->target_y       = target_y;
    tracker->active         = 1;
    tracker->arrived        = 0;
}

int bspline_tracker_next(BSTracker *tracker,
                         float now_x, float now_y,
                         float *out_x, float *out_y)
{
    if (!tracker || !out_x || !out_y) return -1;
    if (!tracker->active) return -1;

    /*
     * 如果当前索引指向最后一个点，检查是否已经到达终点
     */
    if (tracker->current_index >= tracker->path.num_samples - 1)
    {
        /* 已经指向最后一个路径点（即最终目标） */
        *out_x = tracker->target_x;
        *out_y = tracker->target_y;

        /* 检查是否到达 */
        float dx = now_x - tracker->target_x;
        float dy = now_y - tracker->target_y;
        float dist = sqrtf(dx * dx + dy * dy);

        if (dist <= BSPLINE_ARRIVE_THRESH)
        {
            tracker->arrived = 1;
            tracker->active  = 0;
            return 0;  /* 已到达 */
        }

        return 1;  /* 继续向最终目标前进 */
    }

    /*
     * 获取当前目标点
     */
    float cur_target_x = tracker->path.x[tracker->current_index];
    float cur_target_y = tracker->path.y[tracker->current_index];

    /* 计算到当前目标点的距离 */
    float dx = now_x - cur_target_x;
    float dy = now_y - cur_target_y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist <= BSPLINE_ARRIVE_THRESH)
    {
        /* 到达当前目标点，前进到下一个 */
        tracker->current_index++;
    }

    /* 输出当前目标点 */
    *out_x = tracker->path.x[tracker->current_index];
    *out_y = tracker->path.y[tracker->current_index];

    return 1;
}

void bspline_tracker_reset(BSTracker *tracker)
{
    if (!tracker) return;
    tracker->current_index = 0;
    tracker->active        = 0;
    tracker->arrived       = 0;
}

uint8_t bspline_tracker_arrived(const BSTracker *tracker)
{
    if (!tracker) return 0;
    return tracker->arrived;
}


// // === 在 Mid360_Function 中全局声明跟踪器（或静态局部） ===
// static BSTracker bs_tracker;
// static uint8_t   bs_path_planned = 0;

// // === 在 visual_data.i == 1 分支中 ===
// if (visual_data.workl_mode == 1)
// {
//     if (!bs_path_planned)
//     {
//         /* 首次进入：规划 B样条路径 */
//         BSPath path;
//         bspline_plan_path(visual_data.x_map, visual_data.y_map,
//                           chassic_data.target_x, chassic_data.target_y,
//                           NULL, NULL, 0,
//                           &path, BSPLINE_PATH_SAMPLES);
//         bspline_tracker_init(&bs_tracker, &path,
//                              visual_data.x_map, visual_data.y_map,
//                              chassic_data.target_x, chassic_data.target_y);
//         bs_path_planned = 1;
//     }

//     /* 逐点跟踪 */
//     float next_x, next_y;
//     int ret = bspline_tracker_next(&bs_tracker,
//                                    visual_data.x_map, visual_data.y_map,
//                                    &next_x, &next_y);
//     if (ret == 1)
//     {
//         chassic_control_auto(&chassic_data,
//                              visual_data.x_map, visual_data.y_map,
//                              next_x, next_y);
//     }
//     else
//     {
//         /* 到达终点 */
//         bs_path_planned = 0;
//     }
// }
// else
// {
//     /* 模式切换时重置 */
//     bs_path_planned = 0;
//     bspline_tracker_reset(&bs_tracker);
// }

