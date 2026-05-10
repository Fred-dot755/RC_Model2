// written by Fred Xiao
#include "DH.h"
#include <math.h>
#include <stdbool.h>


float theta_offset[4] = {0.0f, 45.0f, 170.0f, 0.0f};
float joint_scale[4]  = {1.0f, 1.0f, -1.0f, 1.0f};

float joint_offset[4] = {0.0f, 180.0f, 180.0f, 0.0f};
// float joint_offset[4] = {0.0f, 0.0f, 0.0f, 0.0f};




//正解
void dh_transform(float a, float alpha, float d, float theta, float T[4][4]) 
{
    float c_theta = cosf(theta);
    float s_theta = sinf(theta);
    float c_alpha = cosf(alpha);
    float s_alpha = sinf(alpha);

    T[0][0] =  c_theta;          T[0][1] = -s_theta * c_alpha;   T[0][2] =  s_theta * s_alpha;   T[0][3] = a * c_theta;
    T[1][0] =  s_theta;          T[1][1] =  c_theta * c_alpha;   T[1][2] = -c_theta * s_alpha;   T[1][3] = a * s_theta;
    T[2][0] =  0.0f;             T[2][1] =  s_alpha;             T[2][2] =  c_alpha;             T[2][3] = d;
    T[3][0] =  0.0f;             T[3][1] =  0.0f;                T[3][2] =  0.0f;                T[3][3] = 1.0f;
}

void mat4_mul(const float A[4][4], const float B[4][4], float C[4][4]) 
{
    for (int i = 0; i < 4; i++) 
    {
        for (int j = 0; j < 4; j++) 
        {
            C[i][j] = 0.0f;
            for (int k = 0; k < 4; k++) 
            {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void computeDHTransform(int angle1, int angle2, int angle3, int* x, int* y, int* z) 
{
    float th1 = (joint_scale[0] * angle1 + theta_offset[0]) * PI / 180.0f;
    float th2 = (joint_scale[1] * angle2 + theta_offset[1]) * PI / 180.0f;
    float th3 = (joint_scale[2] * angle3 + theta_offset[2]) * PI / 180.0f;

    float T[4][4] = 
    {
        {1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1}
    };

    float T_temp[4][4];
    float T_next[4][4];

    dh_transform(0.0f, PI/2.0f, L1, th1, T_temp);
    mat4_mul(T, T_temp, T_next);
    for (int i=0; i<4; i++) for (int j=0; j<4; j++) T[i][j] = T_next[i][j];

    dh_transform(L2, 0.0f, 0.0f, th2, T_temp);
    mat4_mul(T, T_temp, T_next);
    for (int i=0; i<4; i++) for (int j=0; j<4; j++) T[i][j] = T_next[i][j];

    dh_transform(L3, 0.0f, 0.0f, th3, T_temp);
    mat4_mul(T, T_temp, T_next);
    for (int i=0; i<4; i++) for (int j=0; j<4; j++) T[i][j] = T_next[i][j];

    *x = -(int)roundf(T[0][3]);
    *y = (int)roundf(T[1][3]);
    *z = (int)roundf(T[2][3]);
}



//逆解

#include <math.h>
#include <stdbool.h>

// 角度归一化到 [0, 360) 用于底座
static inline float wrapTo360(float angle_deg) 
{
    angle_deg = fmodf(angle_deg, 360.0f);
    if (angle_deg < 0) angle_deg += 360.0f;
    return angle_deg;
}

// 角度归一化到 [-180, 180]
static inline float wrapTo180(float angle_deg) 
{
    angle_deg = fmodf(angle_deg + 180.0f, 360.0f);
    if (angle_deg < 0) angle_deg += 360.0f;
    return angle_deg - 180.0f;
}

static void projectToReachableRegion(float* r, float* d, float max_reach, float min_reach) {
    float distance = sqrtf((*r)*(*r) + (*d)*(*d));
    if (distance > max_reach) {
        *r = (*r) * max_reach / distance;
        *d = (*d) * max_reach / distance;
    } else if (distance < min_reach && distance > 1e-6f) {
        *r = (*r) * min_reach / distance;
        *d = (*d) * min_reach / distance;
    }
}

static void dhAnglesToJointAngles(float a1, float a2, float a3, int* j1, int* j2, int* j3) {
    *j1 = (int)roundf(wrapTo360((a1 - theta_offset[0]) * joint_scale[0]));
    *j2 = (int)roundf((a2 - theta_offset[1]) * joint_scale[1]);
    *j3 = (int)roundf((a3 - theta_offset[2]) * joint_scale[2]);
}



bool inverseKinematics(int target_x, int target_y, int target_z, int* angle1, int* angle2, int* angle3)
{
    float x = -(float)target_x;
    float y = (float)target_y;
    float z = (float)target_z;

    float r = -sqrtf(x*x + y*y); 
    float d = z - L1;

    float max_reach = L2 + L3;
    float min_reach = fabsf(L2 - L3);
    float distance_to_base = sqrtf(x*x + y*y + d*d); 
    bool reachable = true;

    if (distance_to_base > max_reach || distance_to_base < min_reach) {
        float distance = distance_to_base;
        if (distance > max_reach) {
            r = r * max_reach / distance;
            d = d * max_reach / distance;
        } else if (distance < min_reach && distance > 1e-6f) {
            r = r * min_reach / distance;
            d = d * min_reach / distance;
        }
        reachable = false;
    }

    float theta1_rad = atan2f(-y, -x);

    float D = (r*r + d*d - L2*L2 - L3*L3) / (2.0f * L2 * L3);
    if (D > 1.0f) D = 1.0f;
    if (D < -1.0f) D = -1.0f;

    float theta3_rad = acosf(D); 

    float beta = atan2f(d, r);
    float phi = atan2f(L3 * sinf(theta3_rad), L2 + L3 * cosf(theta3_rad));
    float theta2_rad = beta - phi;

    float a1 = theta1_rad * 180.0f / PI;
    float a2 = wrapTo180(theta2_rad * 180.0f / PI);
    float a3 = wrapTo180(theta3_rad * 180.0f / PI);


    float a4 = -(a2 + a3); 

    dhAnglesToJointAngles(a1, a2, a3, angle1, angle2, angle3);
    

    if (*angle2 < 0) {
        *angle2 = 0;
        reachable = false;
    } else if (*angle2 > 150) {
        *angle2 = 150;
        reachable = false;
    }

    if (*angle3 < 0) {
        *angle3 = 0;
        reachable = false;
    } 
    else if (*angle3 > 150) {
        *angle3 = 150;
        reachable = false;
    }
    // else {
    //     // 2. 计算动态上限：90 + angle2
    //     int max_angle3 = 90 + *angle2;
        
    //     // 3. 绝对上限为 170
        // if (max_angle3 > 170) {
        //     max_angle3 = 170;
        // }

        // // 4. 执行上限拦截
        // if (*angle3 > max_angle3) {
        //     *angle3 = max_angle3;
        //     reachable = false;
        // }
    // }
    
    return reachable; 
}
