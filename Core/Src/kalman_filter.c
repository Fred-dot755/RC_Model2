//written by Fred Xiao
#include "kalman_filter.h"


KalmanFilter filter;

void Kalman_Init(KalmanFilter* filter, float init_x, float init_p, 
                 float process_noise, float measure_noise) {
    filter->x = init_x;          
    filter->p = init_p;           
    filter->q = process_noise;    
    filter->r = measure_noise;  
    filter->k = 0;               
}

void Kalman_Predict(KalmanFilter* filter) {

    filter->p = filter->p + filter->q;
}

float Kalman_Update(KalmanFilter* filter, float measure) {
    filter->k = filter->p / (filter->p + filter->r);
    
    filter->x = filter->x + filter->k * (measure - filter->x);
    
    filter->p = (1 - filter->k) * filter->p;
    
    return filter->x;
}

float Kalman_Filter(KalmanFilter* filter, float measure) {
    Kalman_Predict(filter);
    return Kalman_Update(filter, measure);
}


