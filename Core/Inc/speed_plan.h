#ifndef __SPEED_PLAN_H
#define __SPEED_PLAN_H

#include <stdint.h>

typedef struct
{
    float start_x;
    float start_y;
    float target_x;
    float target_y;
    float distance;
    float duration_s;
    uint32_t start_tick_ms;
    uint8_t active;
} speed_plan_t;

void speed_plan_reset(speed_plan_t *plan);
void speed_plan_start(speed_plan_t *plan,
                      float start_x, float start_y,
                      float target_x, float target_y,
                      float max_speed,
                      uint32_t now_ms);
float speed_plan_update(speed_plan_t *plan, uint32_t now_ms, float max_speed);
uint8_t speed_plan_need_replan(const speed_plan_t *plan,
                               float target_x, float target_y,
                               float target_epsilon);

#endif
