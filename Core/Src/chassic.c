//written by Fred Xiao

#include "chassic.h"

Chassic result;

static float abs_float(float value) 
{
    return (value < 0) ? -value : value;
}

void chassic_control(float angle_deg, float speed_mag, float w_rotation)
{
    // 0度是前，90度是左
    float radian = angle_deg * (PI / 180.0f);

    float vx = COEFF_VX * speed_mag * cos(radian);
    float vy = COEFF_VY * speed_mag * sin(radian);
    float w = COEFF_W * w_rotation;


    float val_motor1 = vx - vy - w; // FL (左前)
    float val_motor2 = vx + vy + w; // FR (右前)
    float val_motor3 = vx - vy + w; // RR (右后)
    float val_motor4 = vx + vy - w; // RL (左后)

    float max_val = abs_float(val_motor1);
    if (abs_float(val_motor2) > max_val) max_val = abs_float(val_motor2);
    if (abs_float(val_motor3) > max_val) max_val = abs_float(val_motor3);
    if (abs_float(val_motor4) > max_val) max_val = abs_float(val_motor4);

    if (max_val > MAX_PWM_VALUE) 
    {
        float rate = MAX_PWM_VALUE / max_val;
        val_motor1 *= rate;
        val_motor2 *= rate;
        val_motor3 *= rate;
        val_motor4 *= rate;
    }

    if(now_mood.mood  == 0)
    {
        result.motor1.out = (int16_t)val_motor1;
        result.motor2.out = (int16_t)val_motor2;
        result.motor3.out = (int16_t)val_motor3;
        result.motor4.out = (int16_t)val_motor4;

        result.motor5.out = 0;
        result.motor6.out = 0;
        result.motor7.out = 0;
        result.motor8.out = 0;
    }
    else
    {
        result.motor1.out = (int16_t)val_motor1;
        result.motor2.out = (int16_t)val_motor2;
        result.motor3.out = (int16_t)val_motor3;
        result.motor4.out = (int16_t)val_motor4;
        // result.motor1.out = 0;
        // result.motor2.out = 0;
        // result.motor3.out = 0;
        // result.motor4.out = 0;

        result.motor5.out = -(int16_t)val_motor1*4;
        result.motor6.out = -(int16_t)val_motor2*4;
        result.motor7.out = -(int16_t)val_motor3*4;
        result.motor8.out = -(int16_t)val_motor4*4;
    }
}

void chassic_small(void)
{
    if(now_mood.mood  == 0)
    {
        result.motor5.out = 0;
        result.motor6.out = 0;
        result.motor7.out = 0;
        result.motor8.out = 0;
    }
    else
    {
        result.motor5.out = -3000;
        result.motor6.out = -3000;
        result.motor7.out = -3000;
        result.motor8.out = -3000;
    }
}
