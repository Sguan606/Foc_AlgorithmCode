/*
 * @Author: 星必尘Sguan
 * @Date: 2025-04-12 19:49:17
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2025-04-20 21:56:42
 * @FilePath: \test_2804FocMotor\Hardware\as5600.c
 * @Description: AS5600磁编码器
 * 
 * Copyright (c) 2025 by $JUST, All Rights Reserved. 
 */
#include "as5600.h"

#define M_PI 3.14159265358979323846f
extern I2C_HandleTypeDef hi2c3;

#define AS5600_I2C_Address 0x36
#define AS5600_Reg_H 0x0C
#define AS5600_Reg_L 0x0D

static float prev_angle = 0.0f;
static uint32_t prev_time = 0;
static float prev_velocity = 0.0f;   // 上一次角速度（度/秒）
static uint32_t prev_time_velocity = 0; // 上一次角速度时间戳（ms）

/**
 * @description: 读取AS5600磁编码内部寄存器;
 * @return {*}
 */
uint16_t AS5600_ReadData(void) {
    uint8_t Reg_Data[2];
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c3,
        AS5600_I2C_Address << 1,
        AS5600_Reg_H,
        I2C_MEMADD_SIZE_8BIT,
        Reg_Data,
        2,
        100
    );
    if (status != HAL_OK) return 0xFFFF;
    return ((Reg_Data[0] << 8) | Reg_Data[1]) & 0x0FFF;
}


/**
 * @description: 计算得到磁编码器的角度值;
 * @return {*}
 */
float AS5600_GetAngle(void) {
    uint16_t raw_angle = AS5600_ReadData();
    if (raw_angle == 0xFFFF) return NAN;
    return (raw_angle / 4095.0f) * 360.0f;
}


/**
 * @description: 计算得到角速度;
 * @return {*}
 */
float AS5600_GetVelocity(void) {
    uint32_t current_time = HAL_GetTick();
    float current_angle = AS5600_GetAngle();
    if (isnan(current_angle)) return NAN;

    if (prev_time == 0) {
        prev_angle = current_angle;
        prev_time = current_time;
        return 0.0f;
    }

    float delta_angle = current_angle - prev_angle;
    if (delta_angle > 180.0f) delta_angle -= 360.0f;
    else if (delta_angle < -180.0f) delta_angle += 360.0f;

    float delta_time = (current_time - prev_time) / 1000.0f;
    if (delta_time <= 0.01f) return 0.0f;

    prev_angle = current_angle;
    prev_time = current_time;

    static float filtered_velocity = 0.0f;
    float alpha = 0.3f;
    float raw_velocity = delta_angle / delta_time;
    filtered_velocity = alpha * raw_velocity + (1 - alpha) * filtered_velocity;
    return filtered_velocity;
}


/**
 * @description: 计算获取角加速度值(***误差微分后被放大，数值滤波后，已经不怎么可见了***);
 * @return {*}
 */
float AS5600_GetAcceleration(void) {
    uint32_t current_time = HAL_GetTick();
    float current_velocity = AS5600_GetVelocity();
    if (isnan(current_velocity)) return NAN;
 
    if (prev_time_velocity == 0) {
        prev_velocity = current_velocity;
        prev_time_velocity = current_time;
        return 0.0f;
    }
 
    float delta_velocity = current_velocity - prev_velocity;
    float delta_time = (current_time - prev_time_velocity) / 1000.0f;
 
    if (delta_time <= 0.1f) return 0.0f; // 增加时间阈值
 
    prev_velocity = current_velocity;
    prev_time_velocity = current_time;
 
    static float filtered_acceleration = 0.0f;
    float alpha = 0.5f; // 调整滤波器参数
    float raw_acceleration = delta_velocity / delta_time;
    filtered_acceleration = alpha * raw_acceleration + (1 - alpha) * filtered_acceleration;
    return filtered_acceleration;
}


/**
 * @description: 角度换算to弧度制角度（0~2π）
 * @return {*}
 */
float AS5600_GetAngleRad(void) {
    return AS5600_GetAngle() * (M_PI / 180.0f);
}


/**
 * @description: 角速度换算to弧度制角速度（Rad/s）
 * @return {*}
 */
float AS5600_GetVelocityRad(void) {
    return AS5600_GetVelocity() * (M_PI / 180.0f);
}


/**
 * @description: 角速度换算to转速（转/分钟）
 * @return {*}
 */
float AS5600_GetRPM(void) {
    return AS5600_GetVelocity() / 6.0f;
}

