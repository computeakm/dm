// ekf_attitude.h
#ifndef EKF_ATTITUDE_H
#define EKF_ATTITUDE_H

#include <stdint.h>

typedef struct {
    float q[4]; // quaternion w,x,y,z
    float b[3]; // gyro bias
    float P[6][6]; // error-state covariance (phi(3), bias(3))
    float Q[6][6]; // process noise
    float R[3][3]; // measurement noise (accel)
    float dt;
} EKF_Attitude;

void EKF_Attitude_Init(EKF_Attitude *ekf, float dt);
void EKF_Attitude_Predict(EKF_Attitude *ekf, const float gyro[3]);
void EKF_Attitude_UpdateAccel(EKF_Attitude *ekf, const float accel[3]);

#endif
