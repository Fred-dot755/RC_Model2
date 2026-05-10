// written by Fred Xiao
#ifndef __DH_H
#define __DH_H

#include "main.h"
#include <math.h>
#include <stdbool.h>

#define PI 3.14159265358979323846f

#define L1  100.0f
#define L2  445.0f  // Link 1
#define L3  480.0f  // Link 2

// 初始角度偏移（度）
extern float theta_offset[4];
// 关节角度映射系数（实际角度 = joint_scale * DH角度 + joint_offset）
extern float joint_scale[4];
extern float joint_offset[4];

void dh_transform(float a, float alpha, float d, float theta, float T[4][4]) ;
void mat4_mul(const float A[4][4], const float B[4][4], float C[4][4]);
void computeDHTransform(int angle1, int angle2, int angle3, int* x, int* y, int* z) ;
bool inverseKinematics(int target_x, int target_y, int target_z, int* angle1, int* angle2, int* angle3) ;
int inverseKinematicsAllSolutions(int target_x, int target_y, int target_z, int solutions[4][3]) ;




#endif /* __DH_H */