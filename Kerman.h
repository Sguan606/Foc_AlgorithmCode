#ifndef __KERMAN_H
#define __KERMAN_H

#include "stm32g4xx_hal.h"

typedef struct {
    float velocity;      // 估计的角速度（状态变量）
    float P;            // 估计误差协方差
    float Q;            // 过程噪声（系统不确定性）
    float R;            // 测量噪声（传感器不确定性）
} KalmanFilter;

typedef struct {
    float velocity;      // 速度估计
    float acceleration;  // 加速度估计
    float P[2][2];      // 误差协方差矩阵
    float Q[2];         // 过程噪声 [速度, 加速度]
    float R;            // 测量噪声
} KalmanFilter2D;



extern KalmanFilter kf;  // 告诉编译器 "kf 在其他地方定义"

float Kalman_Update(KalmanFilter* kf, float measurement);
void Kalman2D_Init(KalmanFilter2D* kf, float init_velocity, float init_accel, float Q_vel, float Q_accel, float R);
float Kalman2D_Update(KalmanFilter2D* kf, float measurement, float dt);


#endif //Kerman_H

