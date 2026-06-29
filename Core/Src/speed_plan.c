#include "speed_plan.h"
#include <math.h>

#define SPEED_PLAN_MIN_DURATION_S 1.2f
#define SPEED_PLAN_TARGET_EPSILON 0.001f

static float speed_plan_absf(float value)
{
    return (value >= 0.0f) ? value : -value;
}

void speed_plan_reset(speed_plan_t *plan)
{
    if (plan == 0)
    {
        return;
    }

    plan->start_x = 0.0f;
    plan->start_y = 0.0f;
    plan->target_x = 0.0f;
    plan->target_y = 0.0f;
    plan->distance = 0.0f;
    plan->duration_s = 0.0f;
    plan->start_tick_ms = 0U;
    plan->active = 0U;
}

void speed_plan_start(speed_plan_t *plan,
                      float start_x, float start_y,
                      float target_x, float target_y,
                      float max_speed,
                      uint32_t now_ms)
{
    if (plan == 0)
    {
        return;
    }

    float dx = target_x - start_x;
    float dy = target_y - start_y;
    float distance = sqrtf(dx * dx + dy * dy);

    plan->start_x = start_x;
    plan->start_y = start_y;
    plan->target_x = target_x;
    plan->target_y = target_y;
    plan->distance = distance;
    plan->start_tick_ms = now_ms;

    if (distance <= SPEED_PLAN_TARGET_EPSILON || max_speed <= 0.0f)
    {
        plan->duration_s = SPEED_PLAN_MIN_DURATION_S;
        plan->active = 0U;
        return;
    }

    /*
     * Quintic smoothstep velocity peaks at 1.875 * distance / T.
     * Choose T from max_speed so the generated profile never asks for more.
     */
    plan->duration_s = 1.875f * distance / max_speed;
    if (plan->duration_s < SPEED_PLAN_MIN_DURATION_S)
    {
        plan->duration_s = SPEED_PLAN_MIN_DURATION_S;
    }

    plan->active = 1U;
}

float speed_plan_update(speed_plan_t *plan, uint32_t now_ms, float max_speed)
{
    if (plan == 0 || plan->active == 0U || max_speed <= 0.0f)
    {
        return 0.0f;
    }

    float elapsed_s = (float)(now_ms - plan->start_tick_ms) * 0.001f;
    if (elapsed_s >= plan->duration_s)
    {
        plan->active = 0U;
        return 0.0f;
    }

    float u = elapsed_s / plan->duration_s;
    if (u < 0.0f)
    {
        u = 0.0f;
    }
    else if (u > 1.0f)
    {
        u = 1.0f;
    }

    float u2 = u * u;
    float u3 = u2 * u;
    float u4 = u3 * u;
    float velocity = (30.0f * u2 - 60.0f * u3 + 30.0f * u4) *
                     (plan->distance / plan->duration_s);

    if (velocity > max_speed)
    {
        velocity = max_speed;
    }

    return velocity;
}

uint8_t speed_plan_need_replan(const speed_plan_t *plan,
                               float target_x, float target_y,
                               float target_epsilon)
{
    if (plan == 0 || plan->active == 0U)
    {
        return 1U;
    }

    if (target_epsilon <= 0.0f)
    {
        target_epsilon = SPEED_PLAN_TARGET_EPSILON;
    }

    return (speed_plan_absf(plan->target_x - target_x) > target_epsilon ||
            speed_plan_absf(plan->target_y - target_y) > target_epsilon) ? 1U : 0U;
}
