//written by Fred Xiao
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif



#include "main.h"


#define SAMPLE_COUNT 100




typedef struct {
    float x;
    
    float p;
    
    float k;
    
    float q;
    
    float r;
} KalmanFilter;


extern KalmanFilter filter;


void Kalman_Init(KalmanFilter* filter, float init_x, float init_p, 
                 float process_noise, float measure_noise);


void Kalman_Predict(KalmanFilter* filter);


float Kalman_Update(KalmanFilter* filter, float measure);


float Kalman_Filter(KalmanFilter* filter, float measure);

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_FILTER_H */
