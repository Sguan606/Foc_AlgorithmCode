/*
 * @Author: 星必尘Sguan
 * @Date: 2025-05-03 21:11:50
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-05-06 23:45:29
 * @FilePath: \test_2804FocMotor\Hardware\Kerman.c
 * @Description: Kerman卡尔曼滤波函数的具体实现;
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "Kerman.h"

/**
 * @description: 
 * 如果速度仍有高频波动 → 减小 Q（如 0.0001）。
 * 如果响应延迟明显 → 增大 Q 或 减小 R。
 * @return {*}
 */
KalmanFilter kf = {
    .velocity = 0.0f,  // 初始值设为目标速度
    .P = 1.0f,
    .Q = 0.001f,       // 原0.01 → 0.001更信任预测模型
    .R = 0.3f          // 原0.1 → 0.5更抑制测量噪声
};


float Kalman_Update(KalmanFilter* kf, float measurement) {
    // 1. 预测
    float velocity_pred = kf->velocity;  // 假设角速度变化较慢（无控制输入）
    float P_pred = kf->P + kf->Q;

    // 2. 更新（计算卡尔曼增益）
    float K = P_pred / (P_pred + kf->R);

    // 3. 修正估计值
    kf->velocity = velocity_pred + K * (measurement - velocity_pred);
    kf->P = (1 - K) * P_pred;

    return kf->velocity;
}

void Kalman2D_Init(KalmanFilter2D* kf, float init_velocity, float init_accel, float Q_vel, float Q_accel, float R) {
    kf->velocity = init_velocity;
    kf->acceleration = init_accel;
    kf->P[0][0] = 1.0f;  // 速度方差
    kf->P[0][1] = 0.0f;  // 速度-加速度协方差
    kf->P[1][0] = 0.0f;  // 加速度-速度协方差
    kf->P[1][1] = 1.0f;  // 加速度方差
    kf->Q[0] = Q_vel;    // 速度过程噪声
    kf->Q[1] = Q_accel;  // 加速度过程噪声
    kf->R = R;           // 测量噪声
}

float Kalman2D_Update(KalmanFilter2D* kf, float measurement, float dt) {
    // ===== 1. 预测步骤 =====
    // 更新状态预测
    kf->velocity += kf->acceleration * dt;
    // 更新协方差矩阵预测: P = F * P * F^T + Q
    float F[2][2] = {{1, dt}, {0, 1}};  // 状态转移矩阵
    float P_pred[2][2];
    
    // 计算 F * P
    P_pred[0][0] = F[0][0] * kf->P[0][0] + F[0][1] * kf->P[1][0];
    P_pred[0][1] = F[0][0] * kf->P[0][1] + F[0][1] * kf->P[1][1];
    P_pred[1][0] = F[1][0] * kf->P[0][0] + F[1][1] * kf->P[1][0];
    P_pred[1][1] = F[1][0] * kf->P[0][1] + F[1][1] * kf->P[1][1];
    
    // 计算 (F * P) * F^T + Q
    kf->P[0][0] = P_pred[0][0] * F[0][0] + P_pred[0][1] * F[0][1] + kf->Q[0];
    kf->P[0][1] = P_pred[0][0] * F[1][0] + P_pred[0][1] * F[1][1];
    kf->P[1][0] = P_pred[1][0] * F[0][0] + P_pred[1][1] * F[0][1];
    kf->P[1][1] = P_pred[1][0] * F[1][0] + P_pred[1][1] * F[1][1] + kf->Q[1];

    // ===== 2. 更新步骤 =====
    // 计算卡尔曼增益: K = P * H^T / (H * P * H^T + R)
    float H[2] = {1, 0};  // 测量矩阵 [1, 0]（只观测速度）
    float S = kf->P[0][0] + kf->R;  // 创新协方差
    float K[2] = {kf->P[0][0] / S, kf->P[1][0] / S};  // 卡尔曼增益
    
    // 更新状态估计
    float y = measurement - kf->velocity;  // 测量残差
    kf->velocity += K[0] * y;
    kf->acceleration += K[1] * y;
    
    // 更新协方差矩阵: P = (I - K * H) * P
    kf->P[0][0] -= K[0] * kf->P[0][0];
    kf->P[0][1] -= K[0] * kf->P[0][1];
    kf->P[1][0] -= K[1] * kf->P[0][0];
    kf->P[1][1] -= K[1] * kf->P[0][1];
    
    return kf->velocity;
}


