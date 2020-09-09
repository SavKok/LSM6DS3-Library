/**
  ******************************************************************************
  * @file    KalmanFilter.h
  * @author  Savvas Kokkinidis - sabbaskok@hotmail.com
  * @version V1.0
  * @date    31-July-2020
  * @brief   This file contains the common defines and functions prototypes related
  *          the KalmanFilter
  ******************************************************************************
**/

#ifndef __KALMANFILTER__H__
#define __KALMANFILTER__H__

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
 extern "C" {
#endif

typedef  struct 
{
    float Q_angle; 		/* Process noise variance for the accelerometer */
    float Q_bias; 		/* Process noise variance for the gyro bias */
    float R_measure;	/* Measurement noise variance - this is actually the variance of the measurement noise */
    float angle; 		/* The angle calculated by the Kalman filter - part of the 2x1 state vector */
    float bias; 		/* The gyro bias calculated by the Kalman filter - part of the 2x1 state vector */
    float rate; 		/* Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate */
    float P[2][2];  	/* Error Covariance matrix - This is a 2x2 matrix */
}Kalman_struct;


void Start_Kalman_Algorithm(float executeMaxTicks, float timebase);
float Kalman_GetPitch(void);
float Kalman_GetRoll(void);
void Execute_Kalman_Filter(void);
void Kalman_IncTick(void);
uint32_t Kalman_GetTick(void);
void Kalman_SetFlag(void);
uint8_t Kalman_GetFlag(void);
void Kalman_ClearFlag(void);
    
#ifdef __cplusplus
}
#endif

#endif 

