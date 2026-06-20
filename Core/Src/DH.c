// written by Fred Xiao
#include "DH.h"
#include <math.h>
#include <stdbool.h>


float theta_offset[4] = {0.0f, 30.0f, 180.0f, 0.0f};
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
    *j1 = (int)roundf(wrapTo180((a1 - theta_offset[0]) * joint_scale[0]));
    *j2 = (int)roundf((a2 - theta_offset[1]) * joint_scale[1]);
    *j3 = (int)roundf((a3 - theta_offset[2]) * joint_scale[2]);
}

static bool jointAnglesInLimit(int j2, int j3)
{
    return (j2 >= 0 && j2 <= 170 && j3 >= 0 && j3 <= 170);
}

static void findClosestLimitedJointAngles(float target_r, float target_d, int* j2, int* j3)
{
    float best_error = 1.0e30f;
    int best_j2 = 0;
    int best_j3 = 0;

    for (int cand_j2 = 0; cand_j2 <= 170; cand_j2++)
    {
        float th2 = (joint_scale[1] * cand_j2 + theta_offset[1]) * PI / 180.0f;
        float c2 = cosf(th2);
        float s2 = sinf(th2);

        for (int cand_j3 = 0; cand_j3 <= 170; cand_j3++)
        {
            float th3 = (joint_scale[2] * cand_j3 + theta_offset[2]) * PI / 180.0f;
            float cand_r = L2 * c2 + L3 * cosf(th2 + th3);
            float cand_d = L2 * s2 + L3 * sinf(th2 + th3);
            float err_r = cand_r - target_r;
            float err_d = cand_d - target_d;
            float err = err_r * err_r + err_d * err_d;

            if (err < best_error)
            {
                best_error = err;
                best_j2 = cand_j2;
                best_j3 = cand_j3;
            }
        }
    }

    *j2 = best_j2;
    *j3 = best_j3;
}



bool inverseKinematics(int target_x, int target_y, int target_z, int* angle1, int* angle2, int* angle3)
{
    float x = -(float)target_x;
    float y = (float)target_y;
    float z = (float)target_z;

    float d = z - L1;

    float max_reach = L2 + L3;
    float min_reach = fabsf(L2 - L3);
    float rho = sqrtf(x*x + y*y);
    float max_horizontal_sq = max_reach * max_reach - d * d;
    if (max_horizontal_sq < 0.0f) {
        d = (d > 0.0f) ? max_reach : -max_reach;
        max_horizontal_sq = 0.0f;
    }

    float max_horizontal = sqrtf(max_horizontal_sq);
    if (rho > max_horizontal && rho > 1e-6f) {
        float scale = max_horizontal / rho;
        x *= scale;
        y *= scale;
        rho = max_horizontal;
    }

    float min_horizontal_sq = min_reach * min_reach - d * d;
    if (min_horizontal_sq > 0.0f && rho < sqrtf(min_horizontal_sq) && rho > 1e-6f) {
        float min_horizontal = sqrtf(min_horizontal_sq);
        float scale = min_horizontal / rho;
        x *= scale;
        y *= scale;
        rho = min_horizontal;
    }

    float r = -rho; 

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

    if (!jointAnglesInLimit(*angle2, *angle3)) {
        findClosestLimitedJointAngles(r, d, angle2, angle3);
    }
    
    return true; 
}
